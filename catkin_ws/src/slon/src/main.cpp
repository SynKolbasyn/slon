#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "slon/gps.h"
#include "slon/motors.h"
#include "slon/bt.h"


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
using slon::gps;
using slon::motors;
using slon::bt;


struct Coordinates {
    vector<pair<double, double>> cords;
    long long pos = 0;

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
bool robot_state = 0;
float robot_direction = 0.0;

bool gps_state = false;
double lat = 0.0;
double lon = 0.0;

const double k_speed = 70.0;
const double k_p = 1.0;
const double k_i = 1.0;
const double k_d = 1.0;
double I = 0.0;
auto dt = high_resolution_clock::now();
double prev_angle_err = 0.0;

int lspeed = 0;
int rspeed = 0;


pair<double, double> pid(double dist, double angle);
void compass_listener(const Float32& msg);
void gps_listener(const gps& msg);
void bluetooth_listener(const bt& msg);
void go(int lspeed, int rspeed);
void bt_save();


int main(int argc, char** argv) {
	init(argc, argv, "main");

	NodeHandle nh;

	Subscriber compass_sub = nh.subscribe("compass_data", 1000, compass_listener);
	Subscriber gps_sub = nh.subscribe("gps_data", 1000, gps_listener);
	Subscriber bt_sub = nh.subscribe("recieve_by_bluetooth", 1000, bluetooth_listener);
	Publisher mot_pub = nh.advertise<motors>("motors_control", 1000);
	

	Rate loop_rate(10);
	while (ok()) {
		motors msg;
		msg.l_speed = lspeed;
		msg.r_speed = rspeed;
		mot_pub.publish(msg);
		if (!robot_state) continue;

		double dist_to = coordinates.get_dist_to_next(lat, lon);
		double angle_to = coordinates.get_angle_to_next(lat, lon);
		pair<double, double> speed = pid(dist_to, angle_to);

		go(speed.first, speed.second);

		spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}


pair<double, double> pid(double dist, double angle) {
	auto now = high_resolution_clock::now();
	double speed = dist * k_speed;
	double angle_err = angle - robot_direction;
	double P = angle_err * k_p;
	I += angle_err * (now - dt).count() * k_i;
	double D = (angle_err - prev_angle_err) / (now - dt).count() * k_d;
	double U = P + I + D;
	prev_angle_err = angle_err;
	dt = high_resolution_clock::now();
	return pair<double, double> {speed + U, speed - U};
}

void compass_listener(const Float32& msg) {
	robot_direction = msg.data;
	ROS_INFO("COMPASS: %f", robot_direction);
}


void gps_listener(const gps& msg) {
	gps_state = msg.state;
	
}


void bluetooth_listener(const bt& msg) {
	ROS_INFO("BLUETOOTH: %s", msg.data.c_str());
	if (msg.command.compare("Save") == 0) {
		bt_save();
	}

	else if (msg.command.compare("Start") == 0) {
		robot_state = true;
	}

	else if (msg.command.compare("Forward") == 0) {
		go(msg.speed, msg.speed);
	}

	else if (msg.command.compare("Back") == 0) {
		go(-msg.speed, -msg.speed);
	}

	else if (msg.command.compare("Left") == 0) {
		go(-msg.speed, msg.speed);
	}

	else if (msg.command.compare("Right") == 0) {
		go(msg.speed, -msg.speed);
	}

	else if (msg.command.compare("Stop") == 0) {
		go(0, 0);
	}
}


void go(int l_speed, int r_speed) {
	lspeed = l_speed;
	rspeed = r_speed;
}


void bt_save() {
	if (!gps_state) return;
	coordinates.cords.push_back(pair<double, double> {lat, lon});
}
