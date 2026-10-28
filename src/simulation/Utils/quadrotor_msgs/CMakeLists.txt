cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/GoalSet.msg"
  "msg/LQRTrajectory.msg"
  "msg/Odometry.msg"
  "msg/OutputData.msg"
  "msg/PPROutputData.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PositionCommand.msg"
  "msg/SO3Command.msg"
  "msg/Serial.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/TakeoffLand.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
