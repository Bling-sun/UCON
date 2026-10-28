#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <plan_manage/ego_replan_fsm.h>

using namespace fapp_planner;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fapp_planner_node");
  EGOReplanFSM replan_fsm;
  replan_fsm.init(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
