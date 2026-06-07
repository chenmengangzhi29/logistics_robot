#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/set_bool.hpp>
#include "robot_interfaces/action/pick_and_place.hpp"

using PickAndPlace = robot_interfaces::action::PickAndPlace;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickAndPlace>;

class PickPlaceServer : public rclcpp::Node {
public:
    PickPlaceServer() : Node("pick_place_action_server") {
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
    }

    void init_move_group() {
        move_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur5_arm");
        move_->setPlanningTime(5.0);
        move_->setMaxVelocityScalingFactor(0.3);
        move_->setMaxAccelerationScalingFactor(0.3);
    }

private:
    bool moveTo(const geometry_msgs::msg::PoseStamped& pose) {
        move_->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) 
            return false;
        return move_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool setVacuum(bool on) {
        if (!grasp_cli_->wait_for_service(std::chrono::seconds(2)))
            return false;
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = on;
        auto fut = grasp_cli_->async_send_request(req);
        return fut.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
    }

    void execute(const std::shared_ptr<GoalHandle> gh) {
        auto goal = gh->get_goal();
        auto result = std::make_shared<PickAndPlace::Result>();
        auto fb = std::make_shared<PickAndPlace::Feedback>();

        // Pick: approach (z+0.10), grasp, retreat (z+0.10)
        auto approach = goal->pick_pose;
        approach.pose.position.z += 0.10;

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
        if (!moveTo(goal->pick_pose)){
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

        fb->stage = PickAndPlace::Feedback::STAGE_RETREAT;
        fb->progress = 0.5;
        gh->publish_feedback(fb);
        if (!moveTo(approach)){
            result->success = false;
            result->error_code = 4;
            result->error_message = "retreat failed";
            gh->abort(result);
            return;
        }

        // MOVE to above place
        auto place_above = goal->place_pose;
        place_above.pose.position.z += 0.10;
        fb->stage = PickAndPlace::Feedback::STAGE_MOVE;
        fb->progress = 0.6;
        gh->publish_feedback(fb);
        if (!moveTo(place_above)){
            result->success = false;
            result->error_code = 5;
            result->error_message = "move to place failed";
            gh->abort(result);
            return;
        }

        // PLACE descent
        fb->stage = PickAndPlace::Feedback::STAGE_PLACE;
        fb->progress = 0.8;
        gh->publish_feedback(fb);
        if (!moveTo(goal->place_pose)){
            result->success = false;
            result->error_code = 6;
            result->error_message = "place descent failed";
            gh->abort(result);
            return;
        }

        // Release
        if (!setVacuum(false)){
            result->success = false;
            result->error_code = 7;
            result->error_message = "vacuum off failed";
            gh->abort(result);
            return;
        }

        // Retreat & home
        if (!moveTo(place_above)){
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
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceServer>();
    node->init_move_group();
    rclcpp::spin(node);
    return 0;
}