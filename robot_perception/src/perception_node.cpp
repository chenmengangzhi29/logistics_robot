#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <mutex>

#include "robot_perception/aruco_detector.hpp"
#include "robot_perception/yolo_zmq_detector.hpp"
#include "robot_interfaces/msg/detected_object_array.hpp"
#include "robot_interfaces/srv/set_detector_mode.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {
        declare_parameter<std::string>("default_backend", "aruco");
        declare_parameter<double>("marker_length_m", 0.08);
        declare_parameter<std::string>("zmq_endpoint", "tcp://192.168.28.1:5555");
        declare_parameter<std::string>("camera_optical_frame", "camera_color_optical_frame");

        optical_frame_ = get_parameter("camera_optical_frame").as_string();

        aruco_ = std::make_unique<robot_perception::ArucoDetector>(
            robot_perception::ArucoDetector::Config{
                cv::aruco::DICT_4X4_50,
                get_parameter("marker_length_m").as_double()});
        yolo_ = std::make_unique<robot_perception::YoloZmqDetector>(
            robot_perception::YoloZmqDetector::Config{
                get_parameter("zmq_endpoint").as_string(), 1000, 70});

        std::string default_backend = get_parameter("default_backend").as_string();
        if ("yolo_zmq" == default_backend)
        {
            active_ = yolo_.get();
        }
        else
        {
            active_ = aruco_.get();
        }

        auto detect_cbg = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto info_cbg = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions opts;
        opts.callback_group = detect_cbg;
        rclcpp::SubscriptionOptions info_opts;
        info_opts.callback_group = info_cbg;

        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&PerceptionNode::onImage, this, _1), opts);
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10,
            std::bind(&PerceptionNode::onInfo, this, _1), info_opts);

        pub_ = create_publisher<robot_interfaces::msg::DetectedObjectArray>(
            "/perception/detected_objects", 10);

        mode_srv_ = create_service<robot_interfaces::srv::SetDetectorMode>(
            "/perception/set_mode",
            std::bind(&PerceptionNode::onSetMode, this, _1, _2));

        RCLCPP_INFO(get_logger(), "perception_node up, backend=%s", active_->name().c_str());
    }

private:
    void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(info_mtx_);
        last_info_ = *msg;
        have_info_ = true;
    }

    void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        sensor_msgs::msg::CameraInfo info;
        {
            std::lock_guard<std::mutex> lk(info_mtx_);
            if (!have_info_) return;
            info = last_info_;
        }
        cv::Mat bgr;
        try {
            bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what()); return;
        }
        cv::Mat depth; // depth disabled in P1
        auto results = active_->detect(bgr, depth, info);

        robot_interfaces::msg::DetectedObjectArray arr;
        arr.header = msg->header;
        arr.header.frame_id = optical_frame_;
        arr.objects = std::move(results);
        pub_->publish(arr);
    }

    void onSetMode(
        const std::shared_ptr<robot_interfaces::srv::SetDetectorMode::Request> req,
        std::shared_ptr<robot_interfaces::srv::SetDetectorMode::Response> resp) {
            if (req->mode == 0) active_ = aruco_.get();
            else if (req->mode == 1) active_ = yolo_.get();
            else { resp->success = false; resp->current_backend = active_->name(); return; }
            resp->success = true; resp->current_backend = active_->name();
            RCLCPP_INFO(get_logger(), "switched backend -> %s", active_->name().c_str());
    }

    std::unique_ptr<robot_perception::ArucoDetector> aruco_;
    std::unique_ptr<robot_perception::YoloZmqDetector> yolo_;
    robot_perception::DetectorInterface* active_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<robot_interfaces::msg::DetectedObjectArray>::SharedPtr pub_;
    rclcpp::Service<robot_interfaces::srv::SetDetectorMode>::SharedPtr mode_srv_;
    std::mutex info_mtx_;
    sensor_msgs::msg::CameraInfo last_info_;
    bool have_info_ = false;
    std::string optical_frame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}