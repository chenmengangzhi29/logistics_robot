#include "robot_perception/aruco_detector.hpp"
#include <opencv2/calib3d.hpp>
#include <rclcpp/logger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_perception {

ArucoDetector::ArucoDetector(const Config& cfg) : cfg_(cfg) {
    dict_ = cv::aruco::getPredefinedDictionary(cfg_.dictionary_id);
    params_ = cv::aruco::DetectorParameters::create();
}

std::vector<robot_interfaces::msg::DetectedObject>
ArucoDetector::detect(const cv::Mat& rgb, const cv::Mat& /*depth*/,
                        const sensor_msgs::msg::CameraInfo& info) {
    std::vector<robot_interfaces::msg::DetectedObject> out;
    if (rgb.empty()) 
    {
        return out;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(rgb, dict_, corners, ids, params_);
    if (ids.empty())
    {
        return out;
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << 
        info.k[0], info.k[1], info.k[2],
        info.k[3], info.k[4], info.k[5],
        info.k[6], info.k[7], info.k[8]);
    cv::Mat D = cv::Mat(info.d).clone();

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, cfg_.marker_length_m, K, D, rvecs, tvecs);

    for(size_t i = 0; i < ids.size(); ++i) {
        robot_interfaces::msg::DetectedObject obj;
        obj.id = ids[i];
        obj.class_name = "aruco_marker";
        obj.confidence = 1.0f;
        obj.source = 0; // SOURCE_ARUCO
        obj.dimensions.x = cfg_.marker_length_m;
        obj.dimensions.y = cfg_.marker_length_m;
        obj.dimensions.z = 0.001;
        obj.pose.position.x = tvecs[i][0];
        obj.pose.position.y = tvecs[i][1];
        obj.pose.position.z = tvecs[i][2];

        // Rodrigues -> quaternion
        cv::Mat R;
        cv::Rodrigues(rvecs[i], R);
        tf2::Matrix3x3 mat(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
        tf2::Quaternion q;
        mat.getRotation(q);
        obj.pose.orientation.x = q.x();
        obj.pose.orientation.y = q.y();
        obj.pose.orientation.z = q.z();
        obj.pose.orientation.w = q.w();
        out.push_back(obj);
    }
    return out;
}

}   // namespace robot_perception