cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmTarget.action"
)

ament_package()
