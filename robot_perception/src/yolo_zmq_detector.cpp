#include "robot_perception/yolo_zmq_detector.hpp"
#include <opencv2/imgcodecs.hpp>
#include <sstream>

namespace robot_perception {

YoloZmqDetector::YoloZmqDetector(const Config& cfg)
    : cfg_(cfg), ctx_(1), sock_(ctx_, ZMQ_REQ) {
    int t = cfg_.timeout_ms;
    sock_.set(zmq::sockopt::rcvtimeo, t);
    sock_.set(zmq::sockopt::sndtimeo, t);
    sock_.connect(cfg_.endpoint);
}

std::vector<robot_interfaces::msg::DetectedObject>
YoloZmqDetector::detect(const cv::Mat& rgb, const cv::Mat& /*depth*/,
                        const sensor_msgs::msg::CameraInfo& /*info*/) {
    std::vector<robot_interfaces::msg::DetectedObject> out;
    if (rgb.empty()) return out;

    std::vector<uchar> buf;
    cv::imencode(".jpg", rgb, buf,
                {cv::IMWRITE_JPEG_QUALITY, cfg_.jpeg_quality});

    try {
        zmq::message_t req(buf.data(), buf.size());
        if (!sock_.send(req, zmq::send_flags::none)) return out;
        zmq::message_t reply;
        if (!sock_.recv(reply, zmq::recv_flags::none)) return out;

        // Reply format: "id,x,y,z,qx,qy,qz,qw;id,x,y,...;"
        std::string s(static_cast<char*>(reply.data()), reply.size());
        std::stringstream ss(s);
        std::string obj_str;
        while (std::getline(ss, obj_str, ';')) {
            if (obj_str.empty()) continue;
            std::stringstream os(obj_str);
            std::vector<double> v; std::string tok;
            while (std::getline(os, tok, ',')) v.push_back(std::stod(tok));
            if (v.size() != 8) continue;
            robot_interfaces::msg::DetectedObject obj;
            obj.id = static_cast<int>(v[0]);
            obj.class_name = "yolo_class_" + std::to_string(obj.id);
            obj.source = 1;
            obj.confidence = 0.8f;
            obj.pose.position.x = v[1]; obj.pose.position.y = v[2]; obj.pose.position.z = v[3];
            obj.pose.orientation.x = v[4]; obj.pose.orientation.y = v[5];
            obj.pose.orientation.z = v[6]; obj.pose.orientation.w = v[7];
            out.push_back(obj);
        }
    } catch (const zmq::error_t&) {
        return out;
    }
    return out;
}

}   // namespace robot_perception