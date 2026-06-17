#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>

class GripperHalSim : public rclcpp::Node {
public:
    GripperHalSim() : Node("gripper_hal_sim_node") {
        cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        switch_cli_ = create_client<std_srvs::srv::SetBool>("/vacuum_gripper/switch",
                                                            rmw_qos_profile_services_default,
                                                            cbg_);
        srv_ = create_service<std_srvs::srv::SetBool>(
            "/gripper/grasp",
            [this] (std_srvs::srv::SetBool::Request::SharedPtr req,
                              std_srvs::srv::SetBool::Response::SharedPtr resp)
            {
                if (!switch_cli_->wait_for_service(std::chrono::seconds(1))) {
                    resp->success = false;
                    resp->message = "vacuum plugin not available";
                    return;
                }
                auto vacuum_req = std::make_shared<std_srvs::srv::SetBool::Request>();
                vacuum_req->data = req->data;
                auto future = switch_cli_->async_send_request(vacuum_req);
                if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
                    resp->success = false;
                    resp->message = "vacuum plugin switch timed out";
                    return;
                }

                const auto vacuum_resp = future.get();
                resp->success = vacuum_resp->success;
                resp->message = vacuum_resp->message;
            },
            rmw_qos_profile_services_default,
            cbg_);
    }
private:
    rclcpp::CallbackGroup::SharedPtr cbg_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr switch_cli_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperHalSim>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    return 0;
}
