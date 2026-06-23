#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "robot_interfaces/msg/detected_object_array.hpp"

class MarkerPublisher : public rclcpp::Node {
public:
    MarkerPublisher() : Node("detection_marker_publisher") {
        sub_ = create_subscription<robot_interfaces::msg::DetectedObjectArray>(
            "perception/detected_objects", 10,
             std::bind(&MarkerPublisher::publish, this, std::placeholders::_1));
        pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/perception/markers", 10);
    }
private:
    void publish(const robot_interfaces::msg::DetectedObjectArray::SharedPtr m) {
        visualization_msgs::msg::MarkerArray arr;
        int idx = 0;
        for (const auto& obj : m->objects) {
            visualization_msgs::msg::Marker mk;
            mk.header = m->header;
            mk.ns = "detections"; mk.id = idx++;
            mk.type = mk.CUBE; mk.action = mk.ADD;
            mk.pose = obj.pose;
            mk.scale.x = obj.dimensions.x > 0 ? obj.dimensions.x : 0.08;
            mk.scale.y = obj.dimensions.y > 0 ? obj.dimensions.y : 0.08;
            mk.scale.z = 0.01;
            mk.color.r = 1.0; mk.color.a = 0.7;
            mk.lifetime = rclcpp::Duration::from_seconds(0.5);
            arr.markers.push_back(mk);
        }
        pub_->publish(arr);
    }
    rclcpp::Subscription<robot_interfaces::msg::DetectedObjectArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerPublisher>());
    return 0;
}