#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <thread>
#include <vector>

#include "robot_interfaces/action/pick_and_place.hpp"

using PickAndPlace = robot_interfaces::action::PickAndPlace;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickAndPlace>;

class PickPlaceServer : public rclcpp::Node {
public:
    static constexpr const char* kArmGroup = "ur5_arm";
    static constexpr const char* kTcpLink = "vacuum_tcp";
    static constexpr const char* kPlanningTipLink = "ur5_tool0";
    static constexpr double kTool0ToTcpZ = 0.05;

    PickPlaceServer() : Node("pick_place_action_server") {
        declare_parameter<double>("surface_clearance_m", 0.005);
        declare_parameter<bool>("gazebo_attach_fallback", true);
        declare_parameter<bool>("gazebo_attach_disable_vacuum_force", true);
        declare_parameter<double>("gazebo_attach_max_distance_m", 0.25);
        declare_parameter<double>("gazebo_attached_object_z_offset_m", 0.055);
        declare_parameter<int>("gazebo_attached_update_period_ms", 1);
        server_ = rclcpp_action::create_server<PickAndPlace>(
            this, "pick_and_place",
            [] (const rclcpp_action::GoalUUID&, PickAndPlace::Goal::ConstSharedPtr){
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [] (const std::shared_ptr<GoalHandle>){
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> gh){
                std::thread([this, gh](){
                    execute(gh);
                }).detach();
            });
        grasp_cli_ = create_client<std_srvs::srv::SetBool>("/gripper/grasp");
        gazebo_get_entity_cli_ =
            create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
        gazebo_set_entity_cli_ =
            create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
        grasping_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/vacuum_gripper/grasping", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                vacuum_grasping_.store(msg->data);
            });
    }

    void init_move_group() {
        move_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), kArmGroup);
        move_->setPlanningTime(8.0);
        move_->setNumPlanningAttempts(5);
        move_->setEndEffectorLink(kPlanningTipLink);
        move_->setGoalPositionTolerance(0.01);
        move_->setGoalOrientationTolerance(0.05);
        move_->setMaxVelocityScalingFactor(0.3);
        move_->setMaxAccelerationScalingFactor(0.3);
    }

private:
    geometry_msgs::msg::PoseStamped tool0TargetForTcp(
        const geometry_msgs::msg::PoseStamped& tcp_target) const {
        tf2::Transform tcp_tf;
        tf2::fromMsg(tcp_target.pose, tcp_tf);

        tf2::Transform tool0_to_tcp;
        tool0_to_tcp.setIdentity();
        tool0_to_tcp.setOrigin(tf2::Vector3(0.0, 0.0, kTool0ToTcpZ));

        const auto tool0_tf = tcp_tf * tool0_to_tcp.inverse();
        const auto tool0_msg = tf2::toMsg(tool0_tf);

        geometry_msgs::msg::PoseStamped tool0_target;
        tool0_target.header = tcp_target.header;
        tool0_target.pose.position.x = tool0_msg.translation.x;
        tool0_target.pose.position.y = tool0_msg.translation.y;
        tool0_target.pose.position.z = tool0_msg.translation.z;
        tool0_target.pose.orientation = tool0_msg.rotation;
        return tool0_target;
    }

    bool moveTo(const geometry_msgs::msg::PoseStamped& pose) {
        const auto tool0_pose = tool0TargetForTcp(pose);
        move_->stop();
        move_->setStartStateToCurrentState();
        move_->setPoseReferenceFrame(pose.header.frame_id);
        move_->clearPoseTargets();
        move_->setPoseTarget(tool0_pose, kPlanningTipLink);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool planned = false;
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (move_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                planned = true;
                break;
            }
            move_->setStartStateToCurrentState();
        }
        if (!planned)
            return false;
        const bool ok = move_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        move_->stop();
        move_->clearPoseTargets();
        return ok;
    }

    bool moveLinearTo(const geometry_msgs::msg::PoseStamped& pose) {
        const auto tool0_pose = tool0TargetForTcp(pose);
        move_->stop();
        move_->setStartStateToCurrentState();
        move_->setPoseReferenceFrame(pose.header.frame_id);
        move_->clearPoseTargets();
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(tool0_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = move_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
        if (fraction < 0.95)
            return false;

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        const bool ok = move_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        move_->stop();
        move_->clearPoseTargets();
        return ok;
    }

    bool setVacuum(bool on) {
        if (!grasp_cli_->wait_for_service(std::chrono::seconds(2)))
            return false;
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = on;
        auto fut = grasp_cli_->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
            return false;
        return fut.get()->success;
    }

    bool waitForVacuumGrasping(bool expected, std::chrono::milliseconds timeout) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            if (vacuum_grasping_.load() == expected)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return vacuum_grasping_.load() == expected;
    }

    bool getGazeboPose(const std::string& name, geometry_msgs::msg::Pose& pose) {
        if (!gazebo_get_entity_cli_->wait_for_service(std::chrono::seconds(1)))
            return false;

        auto req = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        req->name = name;
        req->reference_frame = "world";
        auto fut = gazebo_get_entity_cli_->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
            return false;

        auto resp = fut.get();
        if (!resp->success)
            return false;
        pose = resp->state.pose;
        return true;
    }

    bool setGazeboPose(const std::string& name, const geometry_msgs::msg::Pose& pose) {
        if (!gazebo_set_entity_cli_->wait_for_service(std::chrono::seconds(1)))
            return false;

        auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        req->state.name = name;
        req->state.reference_frame = "world";
        req->state.pose = pose;
        req->state.twist.linear.x = 0.0;
        req->state.twist.linear.y = 0.0;
        req->state.twist.linear.z = 0.0;
        req->state.twist.angular.x = 0.0;
        req->state.twist.angular.y = 0.0;
        req->state.twist.angular.z = 0.0;

        auto fut = gazebo_set_entity_cli_->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
            return false;
        return fut.get()->success;
    }

    bool updateAttachedObjectPose() {
        if (attached_model_.empty())
            return false;

        geometry_msgs::msg::Pose tcp_pose;
        if (!getGazeboPose("warehouse_robot::vacuum_tcp", tcp_pose)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "failed to read vacuum_tcp pose while following attached object");
            return false;
        }

        geometry_msgs::msg::Pose object_pose;
        object_pose.position = tcp_pose.position;
        object_pose.position.z -= get_parameter("gazebo_attached_object_z_offset_m").as_double();
        object_pose.orientation = attached_object_orientation_;
        const bool ok = setGazeboPose(attached_model_, object_pose);
        if (!ok) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "failed to update attached object pose for %s",
                attached_model_.c_str());
        }
        return ok;
    }

    bool attachNearestGazeboBox() {
        geometry_msgs::msg::Pose tcp_pose;
        if (!getGazeboPose("warehouse_robot::vacuum_tcp", tcp_pose))
            return false;

        const std::vector<std::string> candidates = {
            "box_with_aruco_0",
            "box_with_aruco_1",
            "box_with_aruco_2",
        };

        double best_distance = std::numeric_limits<double>::max();
        std::string best_model;
        geometry_msgs::msg::Pose best_pose;
        for (const auto& name : candidates) {
            geometry_msgs::msg::Pose box_pose;
            if (!getGazeboPose(name, box_pose))
                continue;

            const double dx = box_pose.position.x - tcp_pose.position.x;
            const double dy = box_pose.position.y - tcp_pose.position.y;
            const double dz = box_pose.position.z - tcp_pose.position.z;
            const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance < best_distance) {
                best_distance = distance;
                best_model = name;
                best_pose = box_pose;
            }
        }

        const double max_distance =
            get_parameter("gazebo_attach_max_distance_m").as_double();
        if (best_model.empty() || best_distance > max_distance) {
            RCLCPP_WARN(
                get_logger(), "no Gazebo box within attach range, best=%s distance=%.3f max=%.3f",
                best_model.empty() ? "<none>" : best_model.c_str(),
                best_distance,
                max_distance);
            return false;
        }

        attached_model_ = best_model;
        attached_object_orientation_ = best_pose.orientation;
        RCLCPP_INFO(
            get_logger(), "Gazebo attach fallback selected %s distance=%.3f",
            attached_model_.c_str(), best_distance);
        return updateAttachedObjectPose();
    }

    bool runWithAttachedObjectFollow(const std::function<bool()>& motion) {
        if (attached_model_.empty())
            return motion();

        std::atomic_bool keep_following{true};
        const auto period = std::chrono::milliseconds(
            std::max<int64_t>(
                10,
                get_parameter("gazebo_attached_update_period_ms").as_int()));
        std::thread follower([this, &keep_following, period]() {
            while (rclcpp::ok() && keep_following.load()) {
                updateAttachedObjectPose();
                std::this_thread::sleep_for(period);
            }
        });

        const bool ok = motion();
        keep_following.store(false);
        if (follower.joinable())
            follower.join();

        updateAttachedObjectPose();
        return ok;
    }

    void execute(const std::shared_ptr<GoalHandle> gh) {
        attached_model_.clear();

        auto goal = gh->get_goal();
        auto result = std::make_shared<PickAndPlace::Result>();
        auto fb = std::make_shared<PickAndPlace::Feedback>();

        // Pick: approach (z+0.50), grasp, retreat (z+0.50)
        auto approach = goal->pick_pose;
        approach.pose.position.z += 0.50;
        auto grasp_pose = goal->pick_pose;
        const auto surface_clearance = get_parameter("surface_clearance_m").as_double();
        grasp_pose.pose.position.z += surface_clearance;
        RCLCPP_INFO(
            get_logger(), "pick surface z=%.4f, clearance=%.4f, grasp target z=%.4f",
            goal->pick_pose.pose.position.z, surface_clearance, grasp_pose.pose.position.z);

        fb->stage = PickAndPlace::Feedback::STAGE_APPROACH;
        fb->progress = 0.1;
        gh->publish_feedback(fb);
        if (!moveTo(approach)){
            result->success = false;
            result->error_code = 1;
            result->error_message = "approach plan failed";
            gh->abort(result);
            return;
        }

        fb->stage = PickAndPlace::Feedback::STAGE_GRASP;
        fb->progress = 0.3;
        gh->publish_feedback(fb);
        if (!moveLinearTo(grasp_pose)){
            result->success = false;
            result->error_code = 2;
            result->error_message = "grasp move failed";
            gh->abort(result);
            return;
        }
        if (!setVacuum(true)){
            result->success = false;
            result->error_code = 3;
            result->error_message = "vacuum on failed";
            gh->abort(result);
            return;
        }
        bool ret = waitForVacuumGrasping(true, std::chrono::milliseconds(1500));
        const bool gazebo_attached =
            get_parameter("gazebo_attach_fallback").as_bool() && attachNearestGazeboBox();
        if (!ret && !gazebo_attached){
            result->success = false;
            result->error_code = 9;
            result->error_message = "vacuum on but object not grasped";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "waitForVacuumGrasping ret=%s gazebo_attached=%s",
            ret ? "true" : "false",
            gazebo_attached ? "true" : "false");
        if (gazebo_attached &&
            get_parameter("gazebo_attach_disable_vacuum_force").as_bool()) {
            setVacuum(false);
        }

        fb->stage = PickAndPlace::Feedback::STAGE_RETREAT;
        fb->progress = 0.5;
        gh->publish_feedback(fb);
        if (!runWithAttachedObjectFollow([this, &approach]() { return moveLinearTo(approach); })){
            result->success = false;
            result->error_code = 4;
            result->error_message = "retreat failed";
            gh->abort(result);
            return;
        }

        // MOVE to above place
        auto place_above = goal->place_pose;
        place_above.pose.position.z += 0.50;
        fb->stage = PickAndPlace::Feedback::STAGE_MOVE;
        fb->progress = 0.6;
        gh->publish_feedback(fb);
        if (!runWithAttachedObjectFollow([this, &place_above]() { return moveLinearTo(place_above); })){
            result->success = false;
            result->error_code = 5;
            result->error_message = "move to place failed";
            gh->abort(result);
            return;
        }

        // PLACE descent
        auto release_pose = goal->place_pose;
        release_pose.pose.position.z += get_parameter("surface_clearance_m").as_double();
        fb->stage = PickAndPlace::Feedback::STAGE_PLACE;
        fb->progress = 0.8;
        gh->publish_feedback(fb);
        if (!runWithAttachedObjectFollow([this, &release_pose]() { return moveLinearTo(release_pose); })){
            result->success = false;
            result->error_code = 6;
            result->error_message = "place descent failed";
            gh->abort(result);
            return;
        }

        // Release
        // if (!setVacuum(false)){
        //     result->success = false;
        //     result->error_code = 7;
        //     result->error_message = "vacuum off failed";
        //     gh->abort(result);
        //     return;
        // }
        waitForVacuumGrasping(false, std::chrono::milliseconds(500));
        attached_model_.clear();

        // Retreat & home
        if (!moveLinearTo(place_above)){
            result->success = false;
            result->error_code = 8;
            result->error_message = "post-place retreat failed";
            gh->abort(result);
            return;
        }

        move_->setNamedTarget("ready");
        move_->move();

        fb->progress = 1.0;
        gh->publish_feedback(fb);
        result->success = true;
        result->error_code = 0;
        result->error_message = "pick+place complete";
        gh->succeed(result);
    }

    rclcpp_action::Server<PickAndPlace>::SharedPtr server_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasp_cli_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr gazebo_get_entity_cli_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr gazebo_set_entity_cli_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasping_sub_;
    std::atomic_bool vacuum_grasping_{false};
    std::string attached_model_;
    geometry_msgs::msg::Quaternion attached_object_orientation_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceServer>();
    node->init_move_group();
    rclcpp::spin(node);
    return 0;
}
