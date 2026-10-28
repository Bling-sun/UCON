cmake_minimum_required(VERSION 3.8)
project(path_searching)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plan_env REQUIRED)
find_package(rcutils REQUIRED)

add_library(path_searching SHARED src/dyn_a_star.cpp)
target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(path_searching plan_env rcutils)
target_link_libraries(path_searching Eigen3::Eigen)

install(TARGETS path_searching EXPORT export_path_searching
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 plan_env rcutils)
ament_package()
