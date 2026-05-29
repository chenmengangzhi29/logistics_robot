#pragma once
#include "robot_perception/detector_interface.hpp"
#include <zmq.hpp>
#include <string>

namespace robot_perception {

class YoloZmqDetector : public DetectorInterface {
public:
    struct Config {
        std::string endpoint = "tcp://192.168.28.1:5555";
        int timeout_ms = 1000;
        int jpeg_quality = 70;
    };
    explicit YoloZmqDetector(const Config& cfg);
    std::vector<robot_interfaces::msg::DetectedObject>
    detect(const cv::Mat& rgb, const cv::Mat& depth,
            const sensor_msgs::msg::CameraInfo& info) override;
    std::string name() const override { return "yolo_zmq"; }

private:
    Config cfg_;
    zmq::context_t ctx_;
    zmq::socket_t sock_;
};

}   // namespace robot_perception