cmake_minimum_required(VERSION 3.8)
project(traj_opt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(obj_state_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(traj_utils REQUIRED)

add_library(traj_opt SHARED src/poly_traj_optimizer.cpp)
target_include_directories(traj_opt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(traj_opt
  obj_state_msgs path_searching plan_env rclcpp rcutils traj_utils)
target_link_libraries(traj_opt Eigen3::Eigen)

install(TARGETS traj_opt EXPORT export_traj_opt
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_traj_opt HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 obj_state_msgs path_searching plan_env rclcpp rcutils traj_utils)
ament_package()
