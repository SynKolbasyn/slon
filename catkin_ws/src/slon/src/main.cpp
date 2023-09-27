#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"


using std::stringstream;

using std_msgs::String;

using ros::ok;
using ros::init;
using ros::Rate;
using ros::spinOnce;
using ros::Publisher;
using ros::NodeHandle;


int main(int argc, char** argv) {
    init(argc, argv, "MAIN");
    NodeHandle n;
    Publisher chatter_pub = n.advertise<std_msgs::String>("main", 1000);

    Rate loop_rate(10);

    int count = 0;
    while (ok()) {
        String msg;

        stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);

        spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
