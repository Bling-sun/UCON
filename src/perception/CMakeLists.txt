cmake_minimum_required(VERSION 3.8)
project(perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(obj_state_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io search segmentation surface)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(mapping_node
  src/mapping_node.cpp
  src/mapping_manger.cpp
  src/Hungarian.cpp
)
target_include_directories(mapping_node PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(mapping_node
  geometry_msgs nav_msgs obj_state_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs
)
target_link_libraries(mapping_node ${PCL_LIBRARIES})

install(TARGETS mapping_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "rviz" EXCLUDE)
install(DIRECTORY launch/rviz DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS scripts/camera_init.py scripts/odom_publish.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
