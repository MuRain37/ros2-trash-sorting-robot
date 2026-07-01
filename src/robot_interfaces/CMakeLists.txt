cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmTarget.action"
  "action/ArmPreset.action"
  "action/ExecuteCoveragePath.action"
  "srv/AddCoverageRectangle.srv"
  "srv/GenerateCoveragePath.srv"
  "srv/DumpTrash.srv"
  "srv/SetLiftSteps.srv"
  "srv/SetGripper.srv"
  "srv/SetServoAngle.srv"
  "srv/DumpTrash.srv"
  DEPENDENCIES nav_msgs
)

ament_package()
