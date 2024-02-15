#include <iostream>
#include <vector>
#include <cstdint>
#include <cstdlib>

#include <opencv2/opencv.hpp>
#include "aruco_samples_utility.hpp"

#include "ros/ros.h"
#include "slon/qrcode.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "camera");
    ros::NodeHandle n;
    ros::Publisher qr_code_pos_pub = n.advertise<slon::qrcode>("qr_code_pos", 1000);

    float marker_length = 0.175;

    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    cv::Mat cam_matrix, dist_coeffs;
    if (!readCameraParameters(std::getenv("CAMERA_CALIBRATION_DATA_PATH"), cam_matrix, dist_coeffs)) {
        std::cerr << "[ ERROR ] -> can not read camera calibration data" << std::endl;
        return -1;
    }

    cv::VideoCapture video(0);
    if (!video.isOpened()) {
        std::cerr << "[ ERROR ] -> can not open video" << std::endl;
        video.release();
        return -1;
    }
    video.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	video.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Set coordinate system
    cv::Mat object_points(4, 1, CV_32FC3);
    object_points.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_length / 2.0, marker_length / 2.0, 0);
    object_points.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_length / 2.0, marker_length / 2.0, 0);
    object_points.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_length / 2.0, -marker_length / 2.0, 0);
    object_points.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_length / 2.0, -marker_length / 2.0, 0);

    while (ros::ok()) {
        cv::Mat frame, gray;
        if (!video.read(frame)) {
            std::cerr << "[ ERROR ] -> can not read frame" << std::endl;
            // video.release();
            // cv::destroyAllWindows();
            // return -1;
            continue;
        }
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int32_t> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        // detect markers and estimate pose
        detector.detectMarkers(gray, corners, ids, rejected);

        std::vector<cv::Vec3d> rvecs(corners.size()), tvecs(corners.size());

        if (!ids.empty()) {
            // Calculate pose for each marker
            for (uint64_t  i = 0; i < corners.size(); ++i) cv::solvePnP(object_points, corners.at(i), cam_matrix, dist_coeffs, rvecs.at(i), tvecs.at(i));
        }

        // draw results
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            for (uint64_t i = 0; i < ids.size(); ++i) {
                if (ids[i] != 0) continue;
                cv::drawFrameAxes(frame, cam_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length, 2);
                slon::qrcode qr_pos;
                qr_pos.x = tvecs[i][0];
                qr_pos.y = tvecs[i][1];
                qr_pos.dist = tvecs[i][2] * 100;
                qr_code_pos_pub.publish(qr_pos);
                ROS_INFO("(%f, %f) -> %f", qr_pos.x, qr_pos.y, qr_pos.dist);
            }
        }
        else {
            slon::qrcode qr_pos;
            qr_pos.x = 0.5;
            qr_pos.y = 0.0;
            qr_pos.dist = 150;
            qr_code_pos_pub.publish(qr_pos);
        }

        imshow("aruco detection", frame);

        if (cv::waitKey(1) == 27) break;

        ros::spinOnce();
    }

    video.release();
    cv::destroyAllWindows();

    return 0;
}
