cmake_minimum_required(VERSION 3.10)
project(robot_control)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_executable(pick_place_action_server src/pick_place_action_server.cpp)
ament_target_dependencies(pick_place_action_server
    rclcpp rclcpp_action moveit_ros_planning_interface std_srvs robot_interfaces)
install(TARGETS pick_place_action_server DESTINATION lib/${PROJECT_NAME})

add_executable(pick_place_mtc_server src/pick_place_mtc_server.cpp)
ament_target_dependencies(pick_place_mtc_server
    rclcpp rclcpp_action moveit_task_constructor_core robot_interfaces)
install(TARGETS pick_place_mtc_server DESTINATION lib/${PROJECT_NAME})

ament_package()