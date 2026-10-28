#include <rclcpp/rclcpp.hpp>
#include "mapping_manager.h"

using namespace mot_mapping;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("mapping_node");

  MappingRos mapping_ros(nh);
  mapping_ros.init();

  rclcpp::spin(nh);
  rclcpp::shutdown();
  
  return 0;
}
