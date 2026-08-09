#include <future>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <thread>
#include <vector>

#include "robot_interfaces/action/pick_and_place.hpp"
#include "robot_interfaces/msg/pick_place_metric.hpp"

using PickAndPlace = robot_interfaces::action::PickAndPlace;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickAndPlace>;

class PickPlaceServer : public rclcpp::Node {
public:
    static constexpr const char* kRobotModelName = "warehouse_robot";
    static constexpr const char* kArmGroup = "ur5_arm";
    static constexpr const char* kTcpLink = "vacuum_tcp";
    static constexpr const char* kBoxLink = "link";
    static constexpr const char* kPlanningTipLink = "ur5_tool0";
    static constexpr double kTool0ToTcpZ = 0.05;

    PickPlaceServer() : Node("pick_place_action_server") {
        declare_parameter<double>("approach_clearance_m", 0.35);
        declare_parameter<double>("surface_clearance_m", 0.015);
        declare_parameter<double>("gazebo_attach_max_distance_m", 0.10);
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
        grasping_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/vacuum_gripper/grasping", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                vacuum_grasping_.store(msg->data);
            });
        attach_cli_ = create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_cli_ = create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
        metrics_pub_ = create_publisher<robot_interfaces::msg::PickPlaceMetric>(
            "/pick_place/metrics", 10);
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
        move_->setMaxAccelerationScalingFactor(0.2);
    }

private:
    void publishMetric(
        const std::string& event_type,
        const std::string& object_name,
        const std::string& planner_type,
        bool success,
        const std::string& error_message,
        const double& duration_ms) {
        robot_interfaces::msg::PickPlaceMetric msg;
        msg.stamp = now();
        msg.event_type = event_type;
        msg.object_name = object_name;
        msg.planner_type = planner_type;
        msg.success = success;
        msg.error_message = error_message;
        msg.duration_ms = duration_ms;
        metrics_pub_->publish(msg);
    }

    // 将TCP坐标转换为tool0坐标
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

    double computeCartesianFractionOnly(const geometry_msgs::msg::PoseStamped& target, const std::string& stage, bool avoid_collisions = true)
    {
        const auto tool0_pose = tool0TargetForTcp(target);
        move_->stop();
        move_->setStartStateToCurrentState();
        move_->setPoseReferenceFrame(target.header.frame_id);
        move_->clearPoseTargets();
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(tool0_pose.pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        moveit_msgs::msg::MoveItErrorCodes error_code;
        const double fraction = move_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, avoid_collisions, &error_code);
        RCLCPP_INFO(get_logger(), "Cartesian check %s fraction=%.3f error_code=%d", stage.c_str(), fraction, error_code.val);
        move_->clearPoseTargets();
        return fraction;
    }

    bool moveToReadyPose() {
        move_->stop();
        move_->setStartStateToCurrentState();
        move_->clearPoseTargets();
        std::map<std::string, double> joints;
        // 硬编码需优化
        joints["ur5_shoulder_pan_joint"] = 0.0;
        joints["ur5_shoulder_lift_joint"] = -1.5708;
        joints["ur5_elbow_joint"] = 1.5708;
        joints["ur5_wrist_1_joint"] = -1.5708;
        joints["ur5_wrist_2_joint"] = -1.5708;
        joints["ur5_wrist_3_joint"] = 0.0;
        move_->setJointValueTarget(joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode error_code;
        error_code = move_->plan(plan);
        if (moveit::core::MoveItErrorCode::SUCCESS != error_code.val) {
            RCLCPP_WARN(get_logger(), "moveToReadyPose plan failed, error_str=%s", moveit::core::error_code_to_string(error_code).c_str());
            return false;
        }
        error_code = move_->execute(plan);
        if (moveit::core::MoveItErrorCode::SUCCESS != error_code.val)
        {
            RCLCPP_WARN(get_logger(), "moveToReadyPose execute failed, error_str=%s", moveit::core::error_code_to_string(error_code).c_str());
            return false;
        }
        move_->stop();
        move_->clearPoseTargets();
        return true;
    }

    // 移动到目标位姿
    bool moveTo(const geometry_msgs::msg::PoseStamped& ompl_pose, const geometry_msgs::msg::PoseStamped& cartesian_pose, const std::string& stage) {
        const auto tool0_pose = tool0TargetForTcp(ompl_pose);
        move_->stop();
        move_->setStartStateToCurrentState();
        move_->setPoseReferenceFrame(ompl_pose.header.frame_id);
        move_->clearPoseTargets();
        move_->setPoseTarget(tool0_pose, kPlanningTipLink);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool result = false;
        for (int attempt = 0; attempt < 3; ++attempt) {
            const auto start = std::chrono::steady_clock::now();
            moveit::core::MoveItErrorCode error_code;
            error_code = move_->plan(plan);
            const bool plan_ok = moveit::core::MoveItErrorCode::SUCCESS == error_code.val;
            const auto stop = std::chrono::steady_clock::now();
            const auto duration_ms =
                std::chrono::duration<double, std::milli>(stop - start).count();
            publishMetric(
                "plan",
                "",
                "ompl",
                plan_ok,
                plan_ok ? "" : "OMPL planning failed",
                duration_ms);
            if (!plan_ok) {
                RCLCPP_INFO(get_logger(), "moveTo %s plan failed, error_code = %s", stage.c_str(), moveit::core::error_code_to_string(error_code).c_str());
                move_->setStartStateToCurrentState();
                continue;
            }
            error_code = move_->execute(plan);
            const bool ok = moveit::core::MoveItErrorCode::SUCCESS == error_code.val;
            move_->stop();
            move_->clearPoseTargets();
            if (!ok)
            {
                RCLCPP_INFO(get_logger(), "moveTo %s execute failed, error_code = %s", stage.c_str(), moveit::core::error_code_to_string(error_code).c_str());
                continue;
            }
            const double fraction = computeCartesianFractionOnly(cartesian_pose, stage);
            const bool path_ok = fraction >= 0.96;
            if (path_ok)
            {
                result = true;
                break;
            }
            RCLCPP_INFO(get_logger(), "moveTo %s computeCartesianFractionOnly failed, fraction=%.3f", stage.c_str(), fraction);
            moveToReadyPose();
        }
        if (!result)
        {
            RCLCPP_ERROR(get_logger(), "moveTo %s attempt failed", stage.c_str());
            return false;
        }
        return true;
    }

    // 笛卡尔轨迹时间参数化
    bool timeParameterizeTrajectory(moveit_msgs::msg::RobotTrajectory& trajectory, const double& velocity_scaling, const double& acceleration_scaling ) {
        auto current_state = move_->getCurrentState(2.0);
        if (!current_state)
        {
            RCLCPP_ERROR(get_logger(), "failed to get current robot state");
            return false;
        }
        robot_trajectory::RobotTrajectory rt(move_->getRobotModel(), kArmGroup);
        rt.setRobotTrajectoryMsg(*move_->getCurrentState(), trajectory);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (!totg.computeTimeStamps(rt, velocity_scaling, acceleration_scaling)) {
            RCLCPP_ERROR(get_logger(), "failed to computeTimeStamps");
            return false;
        }
        rt.getRobotTrajectoryMsg(trajectory);
        return true;
    }

    // 线性移动到目标位姿
    bool moveLinearTo(const geometry_msgs::msg::PoseStamped& pose, const std::string& stage) {
        const auto tool0_pose = tool0TargetForTcp(pose);
        move_->stop();
        move_->setStartStateToCurrentState();
        move_->setPoseReferenceFrame(pose.header.frame_id);
        move_->clearPoseTargets();
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(tool0_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        moveit_msgs::msg::MoveItErrorCodes error_code;
        const auto start = std::chrono::steady_clock::now();
        const double fraction = move_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, true, &error_code);
        const auto stop = std::chrono::steady_clock::now();
        const auto duration_ms =
            std::chrono::duration<double, std::milli>(stop - start).count();
        const bool path_ok = fraction >= 0.80;
        if (!path_ok)
        {
            RCLCPP_ERROR(get_logger(), "moveLinearTo %s computeCartesianPath failed, fraction=%.3f error_code=%d", stage.c_str(), fraction, error_code.val);
            return false;
        }
        if (!timeParameterizeTrajectory(trajectory, 0.9, 0.8)) {
            RCLCPP_ERROR(get_logger(), "cartesian trajectory time parameterization failed");
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        moveit::core::MoveItErrorCode core_error_code = move_->execute(plan);
        const bool ok = moveit::core::MoveItErrorCode::SUCCESS == core_error_code.val;
        move_->stop();
        move_->clearPoseTargets();
        publishMetric(
            "plan",
            "",
            "cartesian",
            path_ok,
            path_ok ? "" : "Cartesian path fraction below threshold",
            duration_ms);
        if (!ok)
        {
            RCLCPP_ERROR(get_logger(), "moveLinearTo %s execute failed, core_error_str=%s", stage.c_str(), moveit::core::error_code_to_string(core_error_code).c_str());
            return false;
        }
        return true;
    }

    bool setTCPAttach(const bool& on)
    {
        if(on)
        {
            if (!attach_cli_->wait_for_service(std::chrono::seconds(2)))
            {
                RCLCPP_ERROR(get_logger(), "setTCPAttach %s wait_for_service failed", on ? "on" : "off");
                return false;
            }
            auto req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
            req->model1_name = kRobotModelName;
            req->link1_name = kTcpLink;
            req->model2_name = attached_model_;
            req->link2_name = kBoxLink;
            auto fut = attach_cli_->async_send_request(req);
            std::future_status status = fut.wait_for(std::chrono::seconds(5));
            if (status != std::future_status::ready)
            {
                RCLCPP_ERROR(get_logger(), "setTCPAttach %s fut wait_for failed, status=%d", on ? "on" : "off", static_cast<int>(status));
                return false;
            }
            return fut.get()->success;
        }
        else 
        {
            if (!detach_cli_->wait_for_service(std::chrono::seconds(2)))
            {
                RCLCPP_ERROR(get_logger(), "setTCPAttach %s wait_for_service failed", on ? "on" : "off");
                return false;
            }
            auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
            req->model1_name = kRobotModelName;
            req->link1_name = kTcpLink;
            req->model2_name = attached_model_;
            req->link2_name = kBoxLink;
            auto fut = detach_cli_->async_send_request(req);
            std::future_status status = fut.wait_for(std::chrono::seconds(5));
            if (status != std::future_status::ready)
            {
                RCLCPP_ERROR(get_logger(), "setTCPAttach %s fut wait_for failed, status=%d", on ? "on" : "off", static_cast<int>(status));
                return false;
            }
            return fut.get()->success;
        }
    }

    // 获取Gazebo实体状态
    bool getGazeboPose(const std::string& name, geometry_msgs::msg::Pose& pose) {
        if (!gazebo_get_entity_cli_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(get_logger(), "getGazeboPose %s wait_for_service failed", name.c_str());
            return false;
        }

        auto req = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        req->name = name;
        req->reference_frame = "world";
        auto fut = gazebo_get_entity_cli_->async_send_request(req);
        std::future_status status = fut.wait_for(std::chrono::seconds(5));
        if (std::future_status::ready != status)
        {
            RCLCPP_ERROR(get_logger(), "getGazeboPose %s fut wait_for failed, status = %d", name.c_str(), static_cast<int>(status));
            return false;
        }

        auto resp = fut.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(get_logger(), "getGazeboPose %s resp->success failed", name.c_str());
            return false;
        }
        pose = resp->state.pose;
        return true;
    }

    // 附属距离tcp最近的Gazebo盒子
    bool attachNearestGazeboBox() {
        geometry_msgs::msg::Pose tcp_pose;
        if (!getGazeboPose("warehouse_robot::vacuum_tcp", tcp_pose))
        {
            RCLCPP_ERROR(get_logger(), "attachNearestGazeboBox getGazeboPose failed");
            return false;
        }

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
                get_logger(), "no Gazebo box within attach range, best_model=%s best_distance=%.3f max_distance=%.3f",
                best_model.empty() ? "<none>" : best_model.c_str(),
                best_distance,
                max_distance);
            return false;
        }

        attached_model_ = best_model;
        attached_model_position_ = best_pose.position;
        attached_object_orientation_ = best_pose.orientation;
        RCLCPP_INFO(
            get_logger(), "Gazebo attach fallback selected %s distance=%.3f",
            attached_model_.c_str(), best_distance);
        return true;
    }

    // 实际执行函数
    void execute(const std::shared_ptr<GoalHandle> gh) {
        const auto all_stage_start = std::chrono::steady_clock::now();

        attached_model_.clear();

        auto goal = gh->get_goal();
        auto result = std::make_shared<PickAndPlace::Result>();
        auto fb = std::make_shared<PickAndPlace::Feedback>();

        // Pick: approach (z+approach_clearance_m), grasp, retreat (z+approach_clearance_m)
        auto approach = goal->pick_pose;
        approach.pose.position.z += get_parameter("approach_clearance_m").as_double();
        auto grasp_pose = goal->pick_pose;
        const auto surface_clearance = get_parameter("surface_clearance_m").as_double();
        grasp_pose.pose.position.z += surface_clearance;
        RCLCPP_INFO(
            get_logger(), "pick surface z=%.4f, clearance=%.4f, grasp target z=%.4f",
            goal->pick_pose.pose.position.z, surface_clearance, grasp_pose.pose.position.z);
 
        // 1.到达抓取前位置
        fb->stage = PickAndPlace::Feedback::STAGE_APPROACH;
        fb->progress = 0.1;
        gh->publish_feedback(fb);
        bool ret = false;
        if(!first_grasp_.load())
        {
            ret = moveTo(approach, grasp_pose, "approach");
            first_grasp_.store(true);
        }
        else
        {
            ret = moveLinearTo(approach, "approach");
        }
        if (!ret){
            RCLCPP_ERROR(get_logger(), "approach move failed");
            result->success = false;
            result->error_code = 1;
            result->error_message = "approach plan failed";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "approach move success");

        // 2.到达抓取位置
        fb->stage = PickAndPlace::Feedback::STAGE_GRASP;
        fb->progress = 0.3;
        gh->publish_feedback(fb);
        if (!moveLinearTo(grasp_pose, "grasp")){
            RCLCPP_ERROR(get_logger(), "grasp move failed");
            result->success = false;
            result->error_code = 2;
            result->error_message = "grasp move failed";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "grasp move success");
        // 3.吸附最近Gazebo盒子
        const bool gazebo_attached = attachNearestGazeboBox();
        if (!gazebo_attached){
            RCLCPP_ERROR(get_logger(), "not object attached");
            result->success = false;
            result->error_code = 9;
            result->error_message = "not object attached";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "attachNearestGazeboBox success");
        
        auto adjusted_pose = grasp_pose;
        adjusted_pose.pose.position.x = attached_model_position_.x;
        adjusted_pose.pose.position.y = attached_model_position_.y;
        if (!moveLinearTo(adjusted_pose, "grasp")){
            RCLCPP_ERROR(get_logger(), "grasp moveLinearTo adjusted_pose failed");
            result->success = false;
            result->error_code = 2;
            result->error_message = "grasp moveLinearTo adjusted_pose failed";
            gh->abort(result);
            return;
        }
        
        // 4.fix_joint模拟抓取
        if (!setTCPAttach(true)){
            RCLCPP_ERROR(get_logger(), "setTCPAttach true failed");
            result->success = false;
            result->error_code = 3;
            result->error_message = "setTCPAttach true failed";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "setTCPAttach true success");

        // 5.返回抓取前位置
        fb->stage = PickAndPlace::Feedback::STAGE_RETREAT;
        fb->progress = 0.5;
        gh->publish_feedback(fb);
        if (!moveLinearTo(approach, "retreat"))
        {
            RCLCPP_ERROR(get_logger(), "retreat move failed");
            result->success = false;
            result->error_code = 4;
            result->error_message = "retreat failed";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "retreat move success");

        // 6.移动到放置前位置
        auto place_above = goal->place_pose;
        place_above.pose.position.z += get_parameter("approach_clearance_m").as_double();
        fb->stage = PickAndPlace::Feedback::STAGE_MOVE;
        fb->progress = 0.6;
        gh->publish_feedback(fb);
        if (!moveLinearTo(place_above, "move_to_place"))
        {
            RCLCPP_ERROR(get_logger(), "move to place failed");
            result->success = false;
            result->error_code = 5;
            result->error_message = "move to place failed";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "move to place success");

        // 7.下降到放置位置
        auto release_pose = goal->place_pose;
        release_pose.pose.position.z += get_parameter("surface_clearance_m").as_double();
        fb->stage = PickAndPlace::Feedback::STAGE_PLACE;
        fb->progress = 0.8;
        gh->publish_feedback(fb);
        if(!moveLinearTo(release_pose, "place_descent"))
        {
            RCLCPP_ERROR(get_logger(), "place descent failed");
            result->success = false;
            result->error_code = 6;
            result->error_message = "place descent failed";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "place descent success");
        // 8.fix_joint释放模拟放置
        if (!setTCPAttach(false)){
            RCLCPP_ERROR(get_logger(), "setTCPAttach false failed");
            result->success = false;
            result->error_code = 7;
            result->error_message = "setTCPAttach false failed";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "setTCPAttach false success");

        // 9.返回放置前位置
        if (!moveLinearTo(place_above, "post_place_retreat")){  
            RCLCPP_ERROR(get_logger(), "post place retreat failed");
            result->success = false;
            result->error_code = 8;
            result->error_message = "post-place retreat failed";
            gh->abort(result);
            return;
        }
        RCLCPP_INFO(
            get_logger(), "post place retreat success");
        const auto all_stage_stop = std::chrono::steady_clock::now();
        const auto all_stage_duration_ms =
            std::chrono::duration<double, std::milli>(all_stage_stop - all_stage_start).count();

        publishMetric(
            "task",
            attached_model_,
            "",
            true,
            "",
            all_stage_duration_ms);

        attached_model_.clear();

        fb->progress = 1.0;
        gh->publish_feedback(fb);
        result->success = true;
        result->error_code = 0;
        result->error_message = "pick+place complete";
        gh->succeed(result);
    }


    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_cli_;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_cli_;

    rclcpp_action::Server<PickAndPlace>::SharedPtr server_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasp_cli_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr gazebo_get_entity_cli_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasping_sub_;
    rclcpp::Publisher<robot_interfaces::msg::PickPlaceMetric>::SharedPtr metrics_pub_;
    std::atomic_bool vacuum_grasping_{false};
    std::atomic_bool first_grasp_{false};
    std::string attached_model_;
    geometry_msgs::msg::Point attached_model_position_;
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
