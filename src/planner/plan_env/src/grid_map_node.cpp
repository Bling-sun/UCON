#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "plan_env/grid_map.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grid_map_node");
  auto grid_map = std::make_shared<GridMap>();
  grid_map->initMap(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
