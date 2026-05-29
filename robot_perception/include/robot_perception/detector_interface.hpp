#pragma once
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "robot_interfaces/msg/detected_object.hpp"

namespace robot_perception {

class DetectorInterface {
public:
    virtual ~DetectorInterface() = default;
    virtual std::vector<robot_interfaces::msg::DetectedObject>
    detect(const cv::Mat& rgb,
           const cv::Mat& depth,
           const sensor_msgs::msg::CameraInfo& info) = 0;
    virtual std::string name() const = 0;
};

}   // namespace robot_perception