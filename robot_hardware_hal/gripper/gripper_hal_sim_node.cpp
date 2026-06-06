#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>

class GripperHalSim : public rclcpp::Node {
public:
    GripperHalSim() : Node("gripper_hal_sim_node") {
        on_cli_ = create_client<std_srvs::srv::Empty>("/vacuum_gripper/on");
        off_cli_ = create_client<std_srvs::srv::Empty>("/vacuum_gripper/off");
        srv_ = create_service<std_srvs::srv::SetBool>(
            "/gripper/grasp",
            [this] (std_srvs::srv::SetBool::Request::SharedPtr req,
                              std_srvs::srv::SetBool::Response::SharedPtr resp)
            {
                auto cli = req->data ? on_cli_ : off_cli_;
                if (!cli->wait_for_service(std::chrono::seconds(1))) {
                    resp->success = false;
                    resp->message = "vacuum plugin not available";
                    return;
                }
                auto fut = cli->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
                resp->success = (fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
                resp->message = req->data ? "vacuum on" : "vacuum off";
            });
    }
private:
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr on_cli_, off_cli_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperHalSim>());
    return 0;
}