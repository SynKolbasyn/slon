#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "slon/gps.h"
#include "slon/motors.h"
#include "slon/bt.h"
#include "slon/qrcode.h"


#define MY_PI 3.1415926535897932384626433832795
#define MY_HALF_PI 1.5707963267948966192313216916398
#define MY_TWO_PI 6.283185307179586476925286766559
#define MY_DEG_TO_RAD 0.017453292519943295769236907684886
#define MY_RAD_TO_DEG 57.295779513082320876798154814105


using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::pair;
using std::chrono::time_point;
using std::chrono::high_resolution_clock;

using ros::ok;
using ros::init;
using ros::Rate;
using ros::spinOnce;
using ros::NodeHandle;
using ros::Subscriber;
using ros::Publisher;

using std_msgs::Float32;
using std_msgs::String;
using std_msgs::Bool;
using slon::gps;
using slon::motors;
using slon::bt;
using slon::qrcode;


bool robot_state = 0;
float robot_direction = 0.0;

bool gps_state = false;
double lat = 0.0;
double lon = 0.0;

const double k_qr_speed = 0.25;
const double k_qr_p = 40.0;
const double k_qr_i = 0.0;
const double k_qr_d = 0.0;
double qr_I = 0.0;
auto qr_dt = high_resolution_clock::now();
double prev_qr_angle_err = 0.0;

int lspeed = 0;
int rspeed = 0;

float qr_dist = 150.0;
float qr_x = 0.0;
float qr_y = 0.0;

auto prev_bt_send_time = high_resolution_clock::now();

bool sprayer_flag = true;


struct Coordinates {
    vector<pair<double, double>> cords;
    long long pos = 0;

	void save_cords() {
		cords.push_back(pair<double, double>{lat, lon});
	}

	pair<double, double> get_next_cords() {
		if (cords.size() >= pos) pos = 0;
		return cords[pos];
	}

    bool set_next_pos() {
        if (++pos >= cords.size()) {
            pos = 0;
            return false;
        }
        return true;
    }

	double get_dist_to_next(double lat1, double long1) {
		double lat2 = get_next_cords().first;
		double long2 = get_next_cords().second;

		double delta = (long1-long2) * MY_DEG_TO_RAD;
		double sdlong = sin(delta);
		double cdlong = cos(delta);
		lat1 = lat1 * MY_DEG_TO_RAD;
		lat2 = lat2 * MY_DEG_TO_RAD;
		double slat1 = sin(lat1);
		double clat1 = cos(lat1);
		double slat2 = sin(lat2);
		double clat2 = cos(lat2);
		delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
		delta = delta * delta;
		delta += clat2 * sdlong * clat2 * sdlong;
		delta = sqrt(delta);
		double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
		delta = atan2(delta, denom);
		return delta * 6372795;
	}

	double get_angle_to_next(double lat1, double long1) {
		double lat2 = get_next_cords().first;
		double long2 = get_next_cords().second;

		double dlon = (long2-long1) * MY_DEG_TO_RAD;
		lat1 = lat1 * MY_DEG_TO_RAD;
		lat2 = lat2 * MY_DEG_TO_RAD;
		double a1 = sin(dlon) * cos(lat2);
		double a2 = sin(lat1) * cos(lat2) * cos(dlon);
		a2 = cos(lat1) * sin(lat2) - a2;
		a2 = atan2(a1, a2);
		if (a2 < 0.0) a2 += MY_TWO_PI;
		return a2 * MY_RAD_TO_DEG;
	}
};


Coordinates coordinates;


pair<double, double> qr_pid();
void compass_listener(const Float32& msg);
void gps_listener(const gps& msg);
void bluetooth_listener(const bt& msg);
void camera_listener(const qrcode& qr_pos);
void go(int lspeed, int rspeed);


int main(int argc, char** argv) {
	init(argc, argv, "main");

	NodeHandle nh;

	Subscriber camera_sub = nh.subscribe("qr_code_pos", 1000, camera_listener);
	Subscriber compass_sub = nh.subscribe("compass_data", 1000, compass_listener);
	Subscriber gps_sub = nh.subscribe("gps_data", 1000, gps_listener);
	Subscriber bt_sub = nh.subscribe("recieve_by_bluetooth", 1000, bluetooth_listener);
	Publisher mot_pub = nh.advertise<motors>("motors_control", 1000);
	Publisher bt_pub = nh.advertise<String>("send_by_bluetooth", 1000);
	Publisher sprayer_pub = nh.advertise<Bool>("sprayer_control", 1000);
	

	Rate loop_rate(10);
	while (ok()) {
		spinOnce();
		loop_rate.sleep();
		
		motors msg;
		msg.l_speed = lspeed;
		msg.r_speed = rspeed;
		mot_pub.publish(msg);
		
		if ((high_resolution_clock::now() - prev_bt_send_time).count() > 1000000000) {
			String gps_d;
			gps_d.data = std::to_string(lat) + ";" + std::to_string(lon);
			bt_pub.publish(gps_d);
			ROS_INFO("BLUETOOTH SEND: %s", gps_d.data.c_str());
			prev_bt_send_time = high_resolution_clock::now();
		}

		if (!robot_state) continue;

		if ((lspeed < 10) && (rspeed < 10) && (sprayer_flag) && (qr_dist < 100)) {
			Bool sprayer_state;
			sprayer_state.data = true;
			for (int i = 0; i < 1000; ++i) sprayer_pub.publish(sprayer_state);
			auto spray_timer = high_resolution_clock::now();
			while ((high_resolution_clock::now() - spray_timer).count() < 1000000000);
			sprayer_state.data = false;
			for (int i = 0; i < 1000; ++i) sprayer_pub.publish(sprayer_state);
			sprayer_flag = false;
		}
		
		pair<double, double> spd = qr_pid();

		lspeed = spd.first;
		rspeed = spd.second;
	}
	
	return 0;
}


void camera_listener(const qrcode& qr_pos) {
	ROS_INFO("QR CODE POS: (%f, %f) -> %f", qr_pos.x, qr_pos.y, qr_pos.dist);
	qr_dist = qr_pos.dist;
	qr_x = qr_pos.x;
	qr_y = qr_pos.y;
}


pair<double, double> qr_pid() {
	auto now = high_resolution_clock::now();
	double speed = qr_dist * k_qr_speed;
	double angle_err = 0.0 - qr_x;
	double P = angle_err * k_qr_p;
	qr_I += angle_err * (now - qr_dt).count() * k_qr_i;
	double D = (angle_err - prev_qr_angle_err) / (now - qr_dt).count() * k_qr_d;
	double U = P + qr_I + D;
	prev_qr_angle_err = angle_err;
	qr_dt = high_resolution_clock::now();
	return pair<double, double> {speed + U, speed - U};
}


void compass_listener(const Float32& msg) {
	robot_direction = msg.data;
	ROS_INFO("COMPASS: %f", robot_direction);
}


void gps_listener(const gps& msg) {
	gps_state = msg.state;
	lat = msg.latitude;
	lon = msg.longitude;
}


void bluetooth_listener(const bt& msg) {
	ROS_INFO("BLUETOOTH: %s", msg.data.c_str());
	if (msg.command.compare("Save") == 0) {
		go(0, 0);
		robot_state = false;
		coordinates.save_cords();
	}

	else if (msg.command.compare("Start") == 0) {
		robot_state = true;
	}

	else if (msg.command.compare("Forward") == 0) {
		go(msg.speed, msg.speed);
		robot_state = false;
	}

	else if (msg.command.compare("Back") == 0) {
		go(-msg.speed, -msg.speed);
		robot_state = false;
	}

	else if (msg.command.compare("Left") == 0) {
		go(-msg.speed, msg.speed);
		robot_state = false;
	}

	else if (msg.command.compare("Right") == 0) {
		go(msg.speed, -msg.speed);
		robot_state = false;
	}

	else if (msg.command.compare("Stop") == 0) {
		go(0, 0);
		robot_state = false;
	}
}


void go(int l_speed, int r_speed) {
	lspeed = l_speed;
	rspeed = r_speed;
}
