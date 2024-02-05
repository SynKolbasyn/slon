#include <iostream>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "slon/qrcode.h"

#include <opencv2/opencv.hpp>


using std::cerr;
using std::endl;
using std::vector;

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

using slon::qrcode;


const int camera_width = 640;
const int camera_heigh = 480;
const float width_to_0_1 = camera_width * 4;
const float heigh_to_0_1 = camera_heigh * 4;

const float real_width = 0.07;
const float focal_length = ((5.0 * 10.0) / real_width); // ((widthInImage * knowDistance) / real_width);


float get_dist_to_qr(double x, double y, double x1, double y1);


int main(int argc, char** argv) {
    init(argc, argv, "camera");
    NodeHandle n;
    Publisher chatter_pub = n.advertise<qrcode>("qr_code_pos", 1000);

    VideoCapture video(0, CAP_ANY);
	
	while (!video.isOpened()) {
		cerr << "ERROR: Can't open video" << endl;
		video = VideoCapture(0, CAP_ANY);
		// video.release();
		// return -1;
	}
	
	video.set(CAP_PROP_FRAME_WIDTH, camera_width);
	video.set(CAP_PROP_FRAME_HEIGHT, camera_heigh);
	
	Mat frame;
	
	QRCodeDetector qcd;
	vector<Point> points;
	Scalar color(0, 0, 255);
	
	Rate loop_rate(10);
	
	while (ok()) {
		if (!video.read(frame)) {
			cerr << "ERROR: Can't read frame" << endl;
			video = VideoCapture(0, CAP_ANY);
			continue;
			// video.release();
			// destroyAllWindows();
			// return -1;
		}

		qcd.detect(frame, points);
		if (points.size() == 4) {
			polylines(frame, points, true, color, 5);
			qrcode qr_pos;
			qr_pos.x = (points[0].x + points[1].x + points[2].x + points[3].x) / width_to_0_1; // avg (1 + 2 + 3 + 4) / 4 / camera width | for diapazon 0 - 1
			qr_pos.y = (points[0].y + points[1].y + points[2].y + points[3].y) / heigh_to_0_1; // Same, but with heigh
			qr_pos.dist = get_dist_to_qr(points[0].x, points[0].y, points[2].x, points[2].y);
            chatter_pub.publish(qr_pos);
			ROS_INFO("(%f, %f) -> %f", qr_pos.x, qr_pos.y, qr_pos.dist);
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


// float get_dist_to_qr(double x, double y, double x1, double y1) {
// 	float eucaldain_dist = sqrt(pow(x1 - x, 2) + pow(y1 - y, 2));
// 	return real_width * focal_length / eucaldain_dist;
// }
float get_dist_to_qr(double x, double y, double x1, double y1) {
	if (((x1 - x) > (camera_width / 4)) && ((y1 - y) > (camera_heigh / 3))) return 50;
	return 200;
}