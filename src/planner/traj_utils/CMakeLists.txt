cmake_minimum_required(VERSION 3.8)
project(traj_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(obj_state_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DataDisp.msg"
  "msg/PolyTraj.msg"
  "msg/MINCOTraj.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

add_library(traj_utils_visualization SHARED src/planning_visualization.cpp)
target_include_directories(traj_utils_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(traj_utils_visualization geometry_msgs obj_state_msgs rclcpp visualization_msgs)
target_link_libraries(traj_utils_visualization Eigen3::Eigen)

install(TARGETS traj_utils_visualization EXPORT export_traj_utils
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_traj_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 geometry_msgs obj_state_msgs rclcpp visualization_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
