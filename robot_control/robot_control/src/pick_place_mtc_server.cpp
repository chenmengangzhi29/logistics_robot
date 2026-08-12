#include <geometry_msgs/msg/detail/vector3_stamped__struct.hpp>
#include <moveit_msgs/msg/detail/move_it_error_codes__struct.hpp>
#include <moveit_task_constructor_msgs/msg/detail/solution__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <rclcpp_action/server.hpp>
#include "robot_interfaces/action/pick_and_place.hpp"

namespace mtc = moveit::task_constructor;
using PickAndPlace = robot_interfaces::action::PickAndPlace;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickAndPlace>;

class MtcPickPlaceServer : public rclcpp::Node {
public:
    MtcPickPlaceServer() : Node("pick_place_mtc_server") {
        server_ = rclcpp_action::create_server<PickAndPlace>(
            this, "pick_and_place_mtc",
            [](const rclcpp_action::GoalUUID&, PickAndPlace::Goal::ConstSharedPtr){
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<GoalHandle>){
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> gh){
                std::thread([this, gh](){
                    execute(gh);
                }).detach();
            }
        );
        solution_pub_ = create_publisher<moveit_task_constructor_msgs::msg::Solution>(
            "/solution", 10);
    }

private:
    void retimeTrajectory(moveit_msgs::msg::RobotTrajectory& traj){
        auto& points = traj.joint_trajectory.points;
        if (points.empty()) return;

        double t = 0.0;
        const double min_dt = 0.2;

        for (size_t i = 0; i < points.size(); ++i){
            t += min_dt;
            int sec = static_cast<int>(t);
            int nanosec = static_cast<int>((t - sec) * 1e9);
            points[i].time_from_start.sec = sec;
            points[i].time_from_start.nanosec = nanosec;
        }
    }

    void retimeSolution(moveit_task_constructor_msgs::msg::Solution& solution) {
        for (auto& sub : solution.sub_trajectory) {
            retimeTrajectory(sub.trajectory);
        }
    }

    mtc::Task buildTask(const geometry_msgs::msg::PoseStamped& pick,
                        const geometry_msgs::msg::PoseStamped& place){
        mtc::Task t;
        t.stages()->setName("warehouse_pick_place");
        t.loadRobotModel(shared_from_this());

        const std::string arm = "ur5_arm";
        const std::string eef = "vacuum";
        const std::string tcp = "vacuum_tcp";

        auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this(), "ompl");
        pipeline->setMaxVelocityScalingFactor(0.1);
        pipeline->setMaxAccelerationScalingFactor(0.1);
        auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian->setStepSize(0.005);
        cartesian->setMinFraction(0.9);
        cartesian->setMaxVelocityScalingFactor(0.1);
        cartesian->setMaxAccelerationScalingFactor(0.1);

        t.setProperty("group", arm);
        t.setProperty("eef", eef);
        t.setProperty("ik_frame", tcp);

        t.add(std::make_unique<mtc::stages::CurrentState>("current"));

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("approach", pipeline);
            stage->setGroup(arm);
            auto p = pick;
            p.pose.position.z += 0.10;
            stage->setGoal(p);
            t.add(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("descend", cartesian);
            stage->setGroup(arm);
            geometry_msgs::msg::Vector3Stamped dir;
            dir.header.frame_id = pick.header.frame_id;
            dir.vector.z = -0.10;
            stage->setDirection(dir);
            t.add(std::move(stage));
        }

        // gripper close stage omitted from MTC - handled via separate vacuum service call after task succeeds;

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian);
            stage->setGroup(arm);
            geometry_msgs::msg::Vector3Stamped dir;
            dir.header.frame_id = pick.header.frame_id;
            dir.vector.z = 0.10;
            stage->setDirection(dir);
            t.add(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("place_above", pipeline);
            stage->setGroup(arm);
            auto p = place;
            p.pose.position.z += 0.10;
            stage->setGoal(p);
            t.add(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("place_descend", cartesian);
            stage->setGroup(arm);
            geometry_msgs::msg::Vector3Stamped dir;
            dir.header.frame_id = place.header.frame_id;
            dir.vector.z = -0.05;
            stage->setDirection(dir);
            t.add(std::move(stage));

        }
        return t;
    } 
    
    void execute(std::shared_ptr<GoalHandle> gh){
        auto goal = gh->get_goal();
        auto task = buildTask(goal->pick_pose, goal->place_pose);
        auto result = std::make_shared<PickAndPlace::Result>();
        try {
            if (!task.plan(5)){
                result->success = false;
                result->error_message = "MTC plan failed";
                gh->abort(result);
                return;
            }
            task.introspection().publishSolution(*task.solutions().front());
            
            using ExecuteTaskSolution =
            moveit_task_constructor_msgs::action::ExecuteTaskSolution;
        
            moveit_task_constructor_msgs::msg::Solution solution_msg;
            task.solutions().front()->toMsg(solution_msg, &task.introspection());
            
            retimeSolution(solution_msg);

            solution_pub_->publish(solution_msg);
            task.introspection().publishSolution(*task.solutions().front());
            
            auto exec_client = rclcpp_action::create_client<ExecuteTaskSolution>(
                shared_from_this(), "/execute_task_solution");
            
            if (!exec_client->wait_for_action_server(std::chrono::seconds(5))) {
            result->success = false;
            result->error_code = 99998;
            result->error_message = "execute_task_solution action server not available";
            gh->abort(result);
            return;
            }
            
            ExecuteTaskSolution::Goal exec_goal;
            exec_goal.solution = solution_msg;
            
            auto goal_future = exec_client->async_send_goal(exec_goal);
            if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
                !goal_future.get()) {
            result->success = false;
            result->error_code = 99997;
            result->error_message = "execute_task_solution goal rejected";
            gh->abort(result);
            return;
            }
            
            result->success = true;
            result->error_code = 0;
            result->error_message = "MTC execute goal accepted";
        } catch (const std::exception& e) {
            result->success = false;
            result->error_message = e.what();
        }
        if (result->success)
            gh->succeed(result);
        else
            gh->abort(result);
    }
    
    rclcpp_action::Server<PickAndPlace>::SharedPtr server_;
    rclcpp::Publisher<moveit_task_constructor_msgs::msg::Solution>::SharedPtr solution_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MtcPickPlaceServer>());
    return 0;
}