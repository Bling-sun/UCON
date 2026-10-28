cmake_minimum_required(VERSION 3.8)
project(fapp_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(obj_state_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(traj_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(fapp_planner_node
  src/ego_planner_node.cpp
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp)
target_include_directories(fapp_planner_node PRIVATE include)
ament_target_dependencies(fapp_planner_node
  geometry_msgs nav_msgs obj_state_msgs path_searching plan_env rclcpp sensor_msgs
  std_msgs traj_opt traj_utils visualization_msgs)
target_link_libraries(fapp_planner_node Eigen3::Eigen)

add_executable(traj_server src/traj_server.cpp)
ament_target_dependencies(traj_server quadrotor_msgs rclcpp std_msgs traj_utils)

install(TARGETS fapp_planner_node traj_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.rviz")
ament_package()
