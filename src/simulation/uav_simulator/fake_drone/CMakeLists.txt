cmake_minimum_required(VERSION 3.8)
project(poscmd_2_odom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(poscmd_2_odom src/poscmd_2_odom.cpp)
ament_target_dependencies(poscmd_2_odom nav_msgs quadrotor_msgs rclcpp visualization_msgs)
target_link_libraries(poscmd_2_odom Eigen3::Eigen)
install(TARGETS poscmd_2_odom DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
ament_package()
