cmake_minimum_required(VERSION 3.8)
project(plan_env)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(plan_env SHARED src/grid_map.cpp src/raycast.cpp)
target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(plan_env
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs
)
target_link_libraries(plan_env Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(grid_map_node src/grid_map_node.cpp)
target_link_libraries(grid_map_node plan_env)
ament_target_dependencies(grid_map_node rclcpp)

install(TARGETS plan_env EXPORT export_plan_env
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS grid_map_node DESTINATION lib/${PROJECT_NAME})
install(FILES
  include/plan_env/grid_map.h
  include/plan_env/raycast.h
  DESTINATION include/plan_env)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)

ament_package()
