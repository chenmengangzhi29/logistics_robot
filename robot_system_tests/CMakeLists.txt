cmake_minimum_required(VERSION 3.10)
project(robot_system_tests)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)

ament_add_gtest_executable(
    pick_place_regression_gtest
    test/pick_place_regression_gtest.cpp
)

ament_target_dependencies(
    pick_place_regression_gtest
    rclcpp
    rclcpp_action
    geometry_msgs
    gazebo_msgs
    robot_interfaces
    ament_index_cpp
    std_srvs
)

install(
    TARGETS pick_place_regression_gtest
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch config results
    DESTINATION share/${PROJECT_NAME}
)

add_launch_test(
    launch/pick_place_regression.test.launch.py
    TIMEOUT 1800
)

ament_package()
