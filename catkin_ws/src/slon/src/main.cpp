#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include <opencv2/opencv.hpp>

#include <boost/algorithm/minmax_element.hpp>
#include <array>
#include <utility>


typedef std::array<int, 4> array;


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

using cv::CAP_ANY;
using cv::CAP_PROP_FRAME_WIDTH;
using cv::CAP_PROP_FRAME_HEIGHT;
using cv::Mat;
using cv::Point;
using cv::imshow;
using cv::Scalar;
using cv::waitKey;
using cv::polylines;
using cv::VideoCapture;
using cv::QRCodeDetector;
using cv::destroyAllWindows;


int main(int argc, char** argv) {
    init(argc, argv, "MAIN");
    NodeHandle n;
    Publisher chatter_pub = n.advertise<Int32MultiArray>("main", 1000);

    VideoCapture video(0, CAP_ANY);
	
	if (!video.isOpened()) {
		cerr << "ERROR: Can't open video" << endl;
		video.release();
		return -1;
	}
	
	video.set(CAP_PROP_FRAME_WIDTH, 640);
	video.set(CAP_PROP_FRAME_HEIGHT, 480);
	
	Mat frame;
	
	QRCodeDetector qcd;
	vector<Point> points;
	Scalar color(0, 0, 255);

    Int32MultiArray arr;
	
	Rate loop_rate(100);
	
	while (ok()) {
		if (!video.read(frame)) {
			cerr << "ERROR: Can't read frame" << endl;
			video.release();
			destroyAllWindows();
			return -1;
		}

		qcd.detect(frame, points);
		if (points.size() == 4) {
			polylines(frame, points, true, color, 5);
			array a{{points[0].x, points[1].x, points[2].x, points[3].x}};
			std::pair<array::iterator, array::iterator> p = boost::minmax_element(a.begin(), a.end());
            arr.data.push_back(*p.first);
            arr.data.push_back(*p.second);
            chatter_pub.publish(arr);
			ROS_INFO("[%i, %i]", arr.data[0], arr.data[1]);
            arr.data.clear();
		}
		points.clear();
		
		imshow("OpenCV Test", frame);

		if (waitKey(1) == 113) break;
		
		spinOnce();
		loop_rate.sleep();
	}
	
	video.release();
	destroyAllWindows();

    return 0;
}
