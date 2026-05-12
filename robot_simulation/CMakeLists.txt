cmake_minimum_required(VERSION 3.5)
project(robot_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
