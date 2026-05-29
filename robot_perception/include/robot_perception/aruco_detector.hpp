#pragma once
#include "robot_perception/detector_interface.hpp"
#include <opencv2/aruco.hpp>

namespace robot_perception {

class ArucoDetector : public DetectorInterface {
public:
    struct Config {
        int dictionary_id = cv::aruco::DICT_4X4_50;
        double marker_length_m = 0.08;  // physical size of printed marker
    };
    explicit ArucoDetector(const Config& cfg);
    std::vector<robot_interfaces::msg::DetectedObject>
    detect(const cv::Mat& rgb, const cv::Mat& depth,
            const sensor_msgs::msg::CameraInfo& info) override;
    std::string name() const override { return "aruco"; }

    private:
        Config cfg_;
        cv::Ptr<cv::aruco::Dictionary> dict_;
        cv::Ptr<cv::aruco::DetectorParameters> params_;
};

}   // namespace robot_perception