#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include <zmq.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class DetectionPoseNode : public rclcpp::Node
{
public:
    DetectionPoseNode() : Node("detection_pose_estimation_node")
    {
        // NAT模式连接Window主机
        string win_ip = "192.168.122.2";
        int port = 5555;
        context_ = zmq::context_t(1);
        socket_ = zmq::socket_t(context_, ZMQ_REQ);
        socket_.connect("tcp://" + win_ip + ":" + to_string(port));

        // 订阅彩色图像
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&DetectionPoseNode::img_callback, this, std::placeholders::_1));

        // 发布位姿
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_object_pose", 10);
    }

private:
    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // ROS图像转OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            Mat img = cv_ptr->image;

            // JPEG压缩
            vector<uchar> buf;
            imencode(".jpg", img, buf, {IMWRITE_JPEG_QUALITY, 70});

            // ZMQ发送图像
            zmq::message_t send_msg(buf.data(), buf.size());
            socket_.send(send_msg, zmq::send_flags::none);

            // 接收位姿
            zmq::message_t recv_msg;
            zmq::recv_result_t res = socket_.recv(recv_msg, zmq::recv_flags::none);
            if (!res) {
                RCLCPP_ERROR(this->get_logger(), "ZMQ 接收失败");
                return;
            }
            string pose_str(static_cast<char*>(recv_msg.data()), recv_msg.size());

            // 解析位姿 x,y,z,qx,qy,qz,qw
            vector<double> pose;
            size_t pos = 0;
            while ((pos = pose_str.find(',')) != string::npos)
            {
                pose.push_back(stod(pose_str.substr(0, pos)));
                pose_str.erase(0, pos + 1);
            }
            pose.push_back(stod(pose_str));
            if (pose.size() != 7) return;

            // 发布ROS位姿
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = msg->header;
            pose_msg.header.frame_id = "camera_link";
            pose_msg.pose.position.x = pose[0];
            pose_msg.pose.position.y = pose[1];
            pose_msg.pose.position.z = pose[2];
            pose_msg.pose.orientation.x = pose[3];
            pose_msg.pose.orientation.y = pose[4];
            pose_msg.pose.orientation.z = pose[5];
            pose_msg.pose.orientation.w = pose[6];
            pose_pub_->publish(pose_msg);
        }
        catch (exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "通信异常：%s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    zmq::context_t context_;
    zmq::socket_t socket_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionPoseNode>());
    rclcpp::shutdown();
    return 0;
}