#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>


using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::stringstream;

using std_msgs::Int32;
using std_msgs::Int32MultiArray;

using ros::ok;
using ros::init;
using ros::Rate;
using ros::spinOnce;
using ros::Publisher;
using ros::NodeHandle;


int main(int argc, char** argv) {
    init(argc, argv, "MAIN");
    NodeHandle n;
    Publisher chatter_pub = n.advertise<Int32MultiArray>("test", 1000);

    Int32MultiArray arr;

	Rate loop_rate(100);
	
	while (ok()) {
		arr.data.push_back(10);
		arr.data.push_back(100);
		chatter_pub.publish(arr);
		ROS_INFO("[%i, %i]", arr.data[0], arr.data[1]);
		arr.data.clear();
        
		spinOnce();
		loop_rate.sleep();
	}
	
    return 0;
}
