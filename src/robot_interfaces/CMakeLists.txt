cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmTarget.action"
  "srv/SetRectangle.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
