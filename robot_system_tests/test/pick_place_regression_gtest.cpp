#include <future>
#include <gtest/gtest.h>

#include <rclcpp/future_return_code.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "robot_interfaces/action/pick_and_place.hpp"
#include "robot_interfaces/msg/pick_place_metric.hpp"

using namespace std::chrono_literals;

using PickAndPlace = robot_interfaces::action::PickAndPlace;
using GoalHandlePickAndPlace = rclcpp_action::ClientGoalHandle<PickAndPlace>;

namespace {

template <typename T>
T get_or_declare(
    const rclcpp::Node::SharedPtr& node,
    const std::string& name,
    const T& default_value) {
    if (!node->has_parameter(name)) {
        node->declare_parameter<T>(name, default_value);
    }
    return node->get_parameter(name).get_value<T>();
}

geometry_msgs::msg::Pose makePose(
    double x,
    double y,
    double z,
    double qx,
    double qy,
    double qz,
    double qw) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

double average(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) /
           static_cast<double>(values.size());
}

std::string csvEscape(const std::string& value) {
    if (value.find_first_of(",\"\n") == std::string::npos) {
        return value;
    }

    std::string escaped = "\"";
    for (const auto ch : value) {
        if (ch == '"') {
            escaped += "\"\"";
        } else {
            escaped += ch;
        }
    }
    escaped += "\"";
    return escaped;
}

}  // namespace

class PickPlaceRegressionTest : public ::testing::Test {
protected:
    struct Row {
        int round = 0;
        std::string box_name;
        bool box_success = false;
        bool task_success = false;
        double task_duration_ms = 0.0;
        double final_error_m = 0.0;
    };

    struct TaskMetric {
        std::string object_name;
        bool success = false;
        std::string error_message;
        double duration_ms = 0.0;
    };

    void SetUp() override {
        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
        node_ = std::make_shared<rclcpp::Node>("pick_place_regression_gtest", options);

        rounds_ = get_or_declare<int>(node_, "rounds", 5);
        action_name_ =
            get_or_declare<std::string>(node_, "action_name", "/pick_and_place");
        action_timeout_s_ = get_or_declare<double>(node_, "action_timeout_s", 60.0);
        startup_timeout_s_ = get_or_declare<double>(node_, "startup_timeout_s", 120.0);
        reset_settle_s_ = get_or_declare<double>(node_, "reset_settle_s", 1.0);
        pose_tolerance_m_ = get_or_declare<double>(node_, "pose_tolerance_m", 0.5);
        frame_id_ = get_or_declare<std::string>(node_, "frame_id", "base_link");
        gripper_id_ = get_or_declare<std::string>(node_, "gripper_id", "vacuum");
        box_names_ = get_or_declare<std::vector<std::string>>(
            node_,
            "box_names",
            {"box_with_aruco_0", "box_with_aruco_1", "box_with_aruco_2"});

        const auto box_initial_pose_x = get_or_declare<std::vector<double>>(node_, "box_initial_pose_x", {});
        const auto box_initial_pose_y = get_or_declare<std::vector<double>>(node_, "box_initial_pose_y", {});
        const auto box_initial_pose_z = get_or_declare<std::vector<double>>(node_, "box_initial_pose_z", {});
        const auto box_place_pose_x = get_or_declare<std::vector<double>>(node_, "box_place_pose_x", {});
        const auto box_place_pose_y = get_or_declare<std::vector<double>>(node_, "box_place_pose_y", {});
        const auto box_place_pose_z = get_or_declare<std::vector<double>>(node_, "box_place_pose_z", {});
        const auto orientation_x =
            get_or_declare<std::vector<double>>(node_, "orientation_x", {});
        const auto orientation_y =
            get_or_declare<std::vector<double>>(node_, "orientation_y", {});
        const auto orientation_z =
            get_or_declare<std::vector<double>>(node_, "orientation_z", {});
        const auto orientation_w =
            get_or_declare<std::vector<double>>(node_, "orientation_w", {});

        const auto expected_count = box_names_.size();
        ASSERT_GT(expected_count, 0u);
        ASSERT_EQ(box_initial_pose_x.size(), expected_count);
        ASSERT_EQ(box_initial_pose_y.size(), expected_count);
        ASSERT_EQ(box_initial_pose_z.size(), expected_count);
        ASSERT_EQ(box_place_pose_x.size(), expected_count);
        ASSERT_EQ(box_place_pose_y.size(), expected_count);
        ASSERT_EQ(box_place_pose_z.size(), expected_count);
        ASSERT_EQ(orientation_x.size(), expected_count);
        ASSERT_EQ(orientation_y.size(), expected_count);
        ASSERT_EQ(orientation_z.size(), expected_count);
        ASSERT_EQ(orientation_w.size(), expected_count);

        initial_poses_.reserve(expected_count);
        place_poses_.reserve(expected_count);
        for (size_t i = 0; i < expected_count; ++i) {
            initial_poses_.push_back(makePose(
                box_initial_pose_x[i],
                box_initial_pose_y[i],
                box_initial_pose_z[i],
                orientation_x[i],
                orientation_y[i],
                orientation_z[i],
                orientation_w[i]));
            place_poses_.push_back(makePose(
                box_place_pose_x[i],
                box_place_pose_y[i],
                box_place_pose_z[i],
                orientation_x[i],
                orientation_y[i],
                orientation_z[i],
                orientation_w[i]));
        }

        action_client_ = rclcpp_action::create_client<PickAndPlace>(node_, action_name_);
        set_entity_client_ =
            node_->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
        get_entity_client_ =
            node_->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");

        metrics_sub_ = node_->create_subscription<robot_interfaces::msg::PickPlaceMetric>(
            "/pick_place/metrics",
            rclcpp::QoS(100),
            [this](const robot_interfaces::msg::PickPlaceMetric::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(metrics_mutex_);

                if (msg->event_type == "plan") {
                    if (msg->success && msg->planner_type == "ompl") {
                        ompl_durations_ms_.push_back(msg->duration_ms);
                    } else if (msg->success && msg->planner_type == "cartesian") {
                        cartesian_durations_ms_.push_back(msg->duration_ms);
                    }
                    return;
                }

                if (msg->event_type == "task") {
                    TaskMetric metric;
                    metric.object_name = msg->object_name;
                    metric.success = msg->success;
                    metric.error_message = msg->error_message;
                    metric.duration_ms = msg->duration_ms;

                    round_task_metrics_.push_back(metric);

                    if (msg->success) {
                        all_task_durations_ms_.push_back(msg->duration_ms);
                    }
                }
            });
        
        auto_pick_reset_client_ =
            node_->create_client<std_srvs::srv::Trigger>("/auto_pick/reset");
        

        executor_.add_node(node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });
    }

    void TearDown() override {
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        if (node_) {
            executor_.remove_node(node_);
        }
    }

    bool waitForSystemReady() {
        const auto timeout = std::chrono::duration<double>(startup_timeout_s_);
        const auto wait_step = 500ms;
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            // const bool action_ready = action_client_->wait_for_action_server(0s);
            const bool set_ready = set_entity_client_->wait_for_service(0s);
            const bool get_ready = get_entity_client_->wait_for_service(0s);
            if (/*action_ready && */set_ready && get_ready) {
                return true;
            }
            std::this_thread::sleep_for(wait_step);
        }

        return /*action_client_->wait_for_action_server(0s) &&*/
               set_entity_client_->wait_for_service(0s) &&
               get_entity_client_->wait_for_service(0s);
    }

    bool resetBox(const std::string& name, const geometry_msgs::msg::Pose& pose) {
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = name;
        request->state.pose = pose;
        request->state.twist.linear.x = 0.0;
        request->state.twist.linear.y = 0.0;
        request->state.twist.linear.z = 0.0;
        request->state.twist.angular.x = 0.0;
        request->state.twist.angular.y = 0.0;
        request->state.twist.angular.z = 0.0;
        request->state.reference_frame = "world";

        auto future = set_entity_client_->async_send_request(request);
        if (future.wait_for(5s) != std::future_status::ready) {
            return false;
        }
        return future.get()->success;
    }

    bool getBoxPose(const std::string& name, geometry_msgs::msg::Pose& pose) {
        auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = name;
        request->reference_frame = "world";

        auto future = get_entity_client_->async_send_request(request);
        if (future.wait_for(5s) != std::future_status::ready) {
            return false;
        }

        const auto response = future.get();
        if (!response->success) {
            return false;
        }
        pose = response->state.pose;
        return true;
    }

    void clearRoundMetrics() {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        round_task_metrics_.clear();
    }

    bool waitForTaskMetrics(size_t expected_count, double timeout_s) {
        const auto deadline = 
            std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);

        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            std::lock_guard<std::mutex> lock(metrics_mutex_);

            size_t finished_count = 0;
            for (const auto& metric : round_task_metrics_) {
                if (metric.success) {
                    ++finished_count;
                }
            }

            if (finished_count >= expected_count) {
                return true;
            }
            if (round_task_metrics_.size() >= expected_count) {
                RCLCPP_ERROR(node_->get_logger(), "round_task_metrics_.size()=%ld >= expected_count=%ld", round_task_metrics_.size(), expected_count);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return false;
    }

    bool resetAutoPick() {
        if (!auto_pick_reset_client_->wait_for_service(std::chrono::seconds(60))) {
            RCLCPP_ERROR(node_->get_logger(), "service /auto_pick/reset not available");
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = auto_pick_reset_client_->async_send_request(request);

        if(future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "timeout calling /auto_pick/reset");
            return false;
        }

        const auto response = future.get();
        if (!response->success) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "auto_pick reset failed: %s",
                response->message.c_str()
            );
            return false;
        }

        return true;
    }

    bool sendPickPlaceGoal(
        const geometry_msgs::msg::PoseStamped& pick,
        const geometry_msgs::msg::PoseStamped& place,
        const std::string& gripper_id,
        PickAndPlace::Result& result,
        double& duration_s) {
        PickAndPlace::Goal goal;
        goal.pick_pose = pick;
        goal.place_pose = place;
        goal.gripper_id = gripper_id;

        const auto start = std::chrono::steady_clock::now();
        auto goal_future = action_client_->async_send_goal(goal);
        if (goal_future.wait_for(std::chrono::duration<double>(action_timeout_s_)) !=
            std::future_status::ready) {
            duration_s = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();
            result.success = false;
            result.error_code = -1;
            result.error_message = "timed out waiting for goal acceptance";
            return false;
        }

        const auto goal_handle = goal_future.get();
        if (!goal_handle) {
            duration_s = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();
            result.success = false;
            result.error_code = -2;
            result.error_message = "goal rejected";
            return false;
        }

        auto result_future = action_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::duration<double>(action_timeout_s_)) !=
            std::future_status::ready) {
            duration_s = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();
            result.success = false;
            result.error_code = -3;
            result.error_message = "timed out waiting for action result";
            return false;
        }

        const GoalHandlePickAndPlace::WrappedResult wrapped_result = result_future.get();
        duration_s = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start)
                         .count();

        if (wrapped_result.result) {
            result = *wrapped_result.result;
        }

        return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
               wrapped_result.result && wrapped_result.result->success;
    }

    

    double positionError(
        const geometry_msgs::msg::Pose& actual,
        const geometry_msgs::msg::Pose& expected) {
        const double dx = actual.position.x - expected.position.x;
        const double dy = actual.position.y - expected.position.y;
        const double dz = actual.position.z - expected.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    geometry_msgs::msg::PoseStamped stampedPose(const geometry_msgs::msg::Pose& pose) {
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header.frame_id = frame_id_;
        stamped.header.stamp = node_->now();
        stamped.pose = pose;
        return stamped;
    }

    void writeSummaryCsv(
        int successful_rounds,
        int successful_boxes,
        double avg_task_duration_ms,
        double avg_ompl_ms,
        double avg_cartesian_ms) {
        std::filesystem::path results_dir;
        try {
            results_dir =
                std::filesystem::path(
                    ament_index_cpp::get_package_share_directory("robot_system_tests")) /
                "results";
            std::filesystem::create_directories(results_dir);
        } catch (const std::exception&) {
            results_dir = std::filesystem::path("/tmp") / "robot_system_tests";
            std::filesystem::create_directories(results_dir);
        }

        const auto rows_path = results_dir / "pick_place_regression_rows.csv";
        const auto summary_path = results_dir / "pick_place_regression_summary.csv";

        std::ofstream rows(rows_path);
        rows << "round,box_name,box_success,task_success,task_duration_ms,final_error_m\n";
        for (const auto& row : rows_) {
            rows << row.round << ','
                 << csvEscape(row.box_name) << ','
                 << (row.box_success ? "true" : "false") << ','
                 << (row.task_success ? "true" : "false") << ','
                 << row.task_duration_ms << ','
                 << row.final_error_m << '\n';
        }

        const int total_boxes = rounds_ * static_cast<int>(box_names_.size());
        const double full_round_success_rate =
            rounds_ == 0 ? 0.0 : static_cast<double>(successful_rounds) / rounds_;
        const double single_box_success_rate =
            total_boxes == 0 ? 0.0 : static_cast<double>(successful_boxes) / total_boxes;

        std::ofstream summary(summary_path);
        summary << "metric,value\n";
        summary << "rounds," << rounds_ << '\n';
        summary << "single_box_total," << total_boxes << '\n';
        summary << "successful_rounds," << successful_rounds << '\n';
        summary << "successful_single_boxes," << successful_boxes << '\n';
        summary << "full_round_success_rate," << full_round_success_rate << '\n';
        summary << "single_box_success_rate," << single_box_success_rate << '\n';
        summary << "average_single_box_duration_ms," << avg_task_duration_ms << '\n';
        summary << "average_ompl_planning_duration_ms," << avg_ompl_ms << '\n';
        summary << "average_cartesian_path_duration_ms," << avg_cartesian_ms << '\n';

        RCLCPP_INFO(
            node_->get_logger(),
            "Regression CSV written to %s and %s",
            rows_path.string().c_str(),
            summary_path.string().c_str());
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;

    rclcpp_action::Client<PickAndPlace>::SharedPtr action_client_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_client_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr get_entity_client_;
    rclcpp::Subscription<robot_interfaces::msg::PickPlaceMetric>::SharedPtr metrics_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr auto_pick_reset_client_;

    int rounds_ = 20;
    std::string action_name_;
    double action_timeout_s_ = 60.0;
    double startup_timeout_s_ = 120.0;
    double reset_settle_s_ = 1.0;
    double pose_tolerance_m_ = 0.5;
    std::string frame_id_;
    std::string gripper_id_;
    std::vector<std::string> box_names_;
    std::vector<geometry_msgs::msg::Pose> initial_poses_;
    std::vector<geometry_msgs::msg::Pose> place_poses_;

    std::vector<Row> rows_;
    std::mutex metrics_mutex_;
    std::vector<TaskMetric> round_task_metrics_;
    std::vector<double> all_task_durations_ms_;
    std::vector<double> ompl_durations_ms_;
    std::vector<double> cartesian_durations_ms_;
};

TEST_F(PickPlaceRegressionTest, AutoPickThreeBoxTwentyRoundRegression) {
    ASSERT_TRUE(waitForSystemReady());

    int successful_rounds = 0;
    int successful_boxes = 0;

    for (int round = 1; round <= rounds_; ++round) {
        // 重置状态
        if (1 != round)
        {
            for (size_t i = 0; i < box_names_.size(); ++i) {
                ASSERT_TRUE(resetBox(box_names_[i], initial_poses_[i]))
                << "failed to reset " << box_names_[i];
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(reset_settle_s_));
    
            // ASSERT_TRUE(resetAutoPick());
            clearRoundMetrics();
        }

        // 等待所有箱子抓取完成
        const double round_timeout_s = action_timeout_s_ * box_names_.size();
        ASSERT_TRUE(waitForTaskMetrics(box_names_.size(), round_timeout_s));

        bool round_success = true;

        std::vector<TaskMetric> round_metrics;
        {
            std::lock_guard<std::mutex> lock(metrics_mutex_);
            round_metrics = round_task_metrics_;
        }

        // 判断箱子抓取成功，并且放置位置在抓取误差范围内
        for (size_t i = 0; i < box_names_.size(); ++i) {
            const auto& box_name = box_names_[i];

            auto metric_it = std::find_if(
                round_metrics.begin(),
                round_metrics.end(),
                [&](const TaskMetric& metric) {
                    return metric.object_name == box_name;
                });

            const bool task_success =
                metric_it != round_metrics.end() && metric_it->success;

            geometry_msgs::msg::Pose final_pose;
            const bool pose_ok = getBoxPose(box_names_[i], final_pose);
            const double final_error =
                pose_ok ? positionError(final_pose, place_poses_[i])
                        : std::numeric_limits<double>::infinity();
            const bool final_pose_ok = pose_ok && final_error < pose_tolerance_m_;
            const bool box_success = task_success && final_pose_ok;

            if (box_success) {
                ++successful_boxes;
            } else {
                round_success = false;
            }

            Row row;
            row.round = round;
            row.box_name = box_names_[i];
            row.box_success = box_success;
            row.task_success = task_success;
            row.task_duration_ms = metric_it->duration_ms;
            row.final_error_m = final_error;
            rows_.push_back(row);

            EXPECT_TRUE(task_success) << box_name;
            EXPECT_TRUE(final_pose_ok)
                << box_name << " final_error=" << final_error;
            EXPECT_LT(final_error, pose_tolerance_m_) << box_names_[i];
        }

        if (round_success) {
            ++successful_rounds;
        }
    }

    // 计算OMPL和笛卡尔路径平均耗时
    double avg_ompl_ms = 0.0;
    double avg_cartesian_ms = 0.0;
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        avg_ompl_ms = average(ompl_durations_ms_);
        avg_cartesian_ms = average(cartesian_durations_ms_);
    }

    writeSummaryCsv(
        successful_rounds,
        successful_boxes,
        average(all_task_durations_ms_),
        avg_ompl_ms,
        avg_cartesian_ms);

    EXPECT_EQ(successful_rounds, rounds_);
    EXPECT_EQ(successful_boxes, rounds_ * static_cast<int>(box_names_.size()));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    const int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
