#include <gtest/gtest.h>
#include "robot_perception/aruco_detector.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

using robot_perception::ArucoDetector;

// Synthesize an image containint a single marker at known camera-frame distance
static cv::Mat make_marker_image(int id, int img_size = 480) {
    cv::Mat marker;
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::drawMarker(dict, id, 200, marker, 1);
    cv::Mat scene(img_size, img_size, CV_8UC1, cv::Scalar(255));
    marker.copyTo(scene(cv::Rect(140, 140, 200, 200)));
    cv::Mat bgr;
    cv::cvtColor(scene, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

static sensor_msgs::msg::CameraInfo make_camera_info(int w, int h) {
    sensor_msgs::msg::CameraInfo info;
    info.width = w; info.height = h;
    info.k = {600.0, 0, w/2.0, 0, 600.0, h/2.0, 0, 0, 1.0};
    info.d = {0, 0, 0, 0, 0};
    info.distortion_model = "plumb_bob";
    return info;
}

TEST(ArucoDetector, DetectsKnownMarker) {
    ArucoDetector det({cv::aruco::DICT_4X4_50, 0.08});
    cv::Mat img = make_marker_image(7);
    auto info = make_camera_info(img.cols, img.rows);
    cv::Mat depth;  // empty for now
    auto results = det.detect(img, depth, info);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].id, 7);
    EXPECT_GT(results[0].confidence, 0.5f);
}

TEST(ArucoDetector, ReturnsEmptyOnBlankImage) {
    ArucoDetector det({cv::aruco::DICT_4X4_50, 0.08});
    cv::Mat img(480, 480, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat depth;
    EXPECT_TRUE(det.detect(img, depth, make_camera_info(480, 480)).empty());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}