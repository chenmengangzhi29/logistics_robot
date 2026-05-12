#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraHalRealNode : public rclcpp::Node
{
public:
    CameraHalRealNode() : Node("camera_hal_real_node")
    {
        real_image_topic_ = this->declare_parameter<std::string>("real_image_topic", "/camera/color/image_raw");
        real_camera_info_topic_ = this->declare_parameter<std::string>("real_camera_info_topic", "/camera/color/camera_info");
        out_image_topic_ = this->declare_parameter<std::string>("out_image_topic", "/camera/image_raw");
        out_camera_info_topic_ = this->declare_parameter<std::string>("out_camera_info_topic", "/camera/camera_info");
        force_frame_id_ = this->declare_parameter<std::string>("force_frame_id", "");

        auto qos = rclcpp::SensorDataQoS();

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_image_topic_, qos);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(out_camera_info_topic_, qos);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            real_image_topic_, qos,
            std::bind(&CameraHalRealNode::onImage, this, std::placeholders::_1));
        
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            real_camera_info_topic_, qos,
            std::bind(&CameraHalRealNode::onCameraInfo, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "camera_hal_real started");
        RCLCPP_INFO(this->get_logger(), "in: image=%s, info=%s",
            real_image_topic_.c_str(), real_camera_info_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "out: image=%s, info=%s",
            out_image_topic_.c_str(), out_camera_info_topic_.c_str());
    }

    private:
    void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto out = *msg;
        if (!force_frame_id_.empty()) {
            out.header.frame_id = force_frame_id_;
        }
        image_pub_->publish(out);
    }

    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        auto out = *msg;
        if (!force_frame_id_.empty()) {
            out.header.frame_id = force_frame_id_;
        }
        camera_info_pub_->publish(out);
    }

    private:

    std::string real_image_topic_;
    std::string real_camera_info_topic_;
    std::string out_image_topic_;
    std::string out_camera_info_topic_;
    std::string force_frame_id_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraHalRealNode>());
    rclcpp::shutdown();
    return 0;
}