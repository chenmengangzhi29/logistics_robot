cmake_minimum_required(VERSION 3.10)
project(robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(robot_interfaces REQUIRED)

include_directories(include)

# Legacy node kept for archival; not wired into launch
add_executable(detection_pose_estimation_node_legacy
    src/legacy/detection_pose_estimation_node_legacy.cpp
)

ament_target_dependencies(detection_pose_estimation_node_legacy
 rclcpp
 sensor_msgs
 geometry_msgs
 cv_bridge
)

target_include_directories(detection_pose_estimation_node_legacy
    PUBLIC
    ${ZMQ_INCLUDE_DIRS}
)

target_link_libraries(detection_pose_estimation_node_legacy
    zmq
    ${OpenCV_LIBRARIES}
)

install(TARGETS
  detection_pose_estimation_node_legacy
  DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   set(ament_cmake_uncrustify_FOUND TRUE)
#   set(ament_cmake_pep257_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
