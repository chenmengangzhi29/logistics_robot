#include <atomic>
#include <chrono>
#include <optional>
#include <unordered_set>
#include <mutex>
#include <functional>
#include <memory>
#include <robot_interfaces/msg/detail/detected_object__struct.hpp>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "robot_interfaces/msg/detected_object_array.hpp"
#include "robot_interfaces/action/pick_and_place.hpp"

using PickAndPlace = robot_interfaces::action::PickAndPlace;
using GoalHandlePickAndPlace =
    rclcpp_action::ClientGoalHandle<PickAndPlace>;
using WrappedResult = GoalHandlePickAndPlace::WrappedResult;

class AutoPick : public rclcpp::Node {
public:
    AutoPick() : Node("auto_pick_node"),
                tf_buf_(get_clock()), tf_listener_(tf_buf_) {
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<double>("min_pick_interval_s", 6.0);
        // declare_parameter<double>("place_x", 0.0);
        // declare_parameter<double>("place_y", -0.5);
        // declare_parameter<double>("place_z", 0.125);
        declare_parameter<double>("pick_surface_clearance_m", -0.01);

        auto cbg = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions opts;
        opts.callback_group = cbg;
        sub_ = create_subscription<robot_interfaces::msg::DetectedObjectArray>(
            "/perception/detected_objects",
            rclcpp::QoS(10),
            [this](robot_interfaces::msg::DetectedObjectArray::SharedPtr msg) {
                onDetect(msg);
            },
            opts);
        client_ = rclcpp_action::create_client<PickAndPlace>(this, "pick_and_place");
        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "/auto_pick/reset",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                if(busy_.load()){
                    response->success = false;
                    response->message = "auto_pick is busy, reject reset";
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    picked_ids_.clear();
                    current_id_ = -1;
                }
                response->success = true;
                response->message = "auto_pick state reset";      
            });
    }

private: 
    // std::optional<robot_interfaces::msg::DetectedObject> 
    // selectObject(const std::vector<robot_interfaces::msg::DetectedObject>& objects) {
    //     std::lock_guard<std::mutex> lock(mutex_);

    //     for (const auto& obj : objects) {
    //         if (picked_ids_.count(obj.id) == 0) {
    //             return obj;
    //         }
    //     }

    //     return std::nullopt;
    // }


    void onDetect(robot_interfaces::msg::DetectedObjectArray::SharedPtr msg) {
        if (msg->objects.empty()) return;
        auto now = this->now();
        if ((now - last_pick_).seconds() < get_parameter("min_pick_interval_s").as_double()) return;

        // auto selected = selectObject(msg->objects);
        // if (!selected.has_value()) {
        //     return;
        // }

        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true)) return;

        // const auto& obj = selected.value();
        const auto& obj = msg->objects.front();
        current_id_ = obj.id;

        geometry_msgs::msg::PoseStamped cam_pose;
        cam_pose.header = msg->header;
        cam_pose.pose = obj.pose;
        geometry_msgs::msg::PoseStamped base_pose;
        try {
            base_pose = tf_buf_.transform(cam_pose, get_parameter("base_frame").as_string(),
                                        tf2::durationFromSec(0.5));
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN(get_logger(), "tf: %s", e.what());
            busy_.store(false);
            return;
        }

        if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            busy_.store(false);
            return;
        }

        PickAndPlace::Goal g;
        g.pick_pose = base_pose;
        // g.pick_pose.pose.position.z += get_parameter("pick_surface_clearance_m").as_double();
        g.pick_pose.pose.orientation.x = 1.0;
        g.pick_pose.pose.orientation.y = 0.0;
        g.pick_pose.pose.orientation.z = 0.0;
        g.pick_pose.pose.orientation.w = 0.0; // tcp pointing straight down
        g.place_pose.header = base_pose.header;
        // g.place_pose.pose.position.x = get_parameter("place_x").as_double();
        // g.place_pose.pose.position.y = get_parameter("place_y").as_double();
        // g.place_pose.pose.position.z = get_parameter("place_z").as_double();
        double pose_x = 0.0;
        if (base_pose.pose.position.y > 0.05){
            pose_x = base_pose.pose.position.y + 0.05;
        }
        else if (base_pose.pose.position.y < -0.05){
            pose_x = base_pose.pose.position.y - 0.05;
        }
        else {
            pose_x = base_pose.pose.position.y;
        }
        g.place_pose.pose.position.x = pose_x;
        g.place_pose.pose.position.y = -base_pose.pose.position.x;
        g.place_pose.pose.position.z = base_pose.pose.position.z;
        g.place_pose.pose.orientation = g.pick_pose.pose.orientation;

        RCLCPP_INFO(get_logger(), "pick_pose: position: {x=%f,y=%f,z=%f}, place_pose: position: {x=%f,y=%f,z=%f}", 
                    g.pick_pose.pose.position.x, g.pick_pose.pose.position.y, g.pick_pose.pose.position.z,
                    g.place_pose.pose.position.x, g.place_pose.pose.position.y, g.place_pose.pose.position.z);

        last_pick_ = now;
        auto opts = rclcpp_action::Client<PickAndPlace>::SendGoalOptions();
        opts.goal_response_callback = [this](const GoalHandlePickAndPlace::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_WARN(get_logger(), "pick goal rejected");
                busy_.store(false);
            }
        };
        opts.result_callback = [this](const WrappedResult & result) {
            RCLCPP_INFO(get_logger(), "pick result: %s code=%d message=%s",
                        result.result->success ? "OK" : "FAIL",
                        result.result->error_code,
                        result.result->error_message.c_str());
            if (result.result && result.result->success) {
                std::lock_guard<std::mutex> lock(mutex_);
                // picked_ids_.insert(current_id_);
            }

            busy_.store(false);
        };
        client_->async_send_goal(g, opts);
    }

    rclcpp::Subscription<robot_interfaces::msg::DetectedObjectArray>::SharedPtr sub_;
    rclcpp_action::Client<PickAndPlace>::SharedPtr client_;
    tf2_ros::Buffer tf_buf_;
    tf2_ros::TransformListener tf_listener_;
    std::atomic<bool> busy_{false};
    rclcpp::Time last_pick_{0,0,RCL_ROS_TIME};

    std::unordered_set<int32_t> picked_ids_;
    int32_t current_id_;
    std::mutex mutex_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto n = std::make_shared<AutoPick>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(n);
    exec.spin();
    return 0;
}
