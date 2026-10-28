#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class PositionCommandToOdometry : public rclcpp::Node
{
public:
  PositionCommandToOdometry() : Node("poscmd_2_odom")
  {
    init_x_ = declare_parameter<double>("init_x", 0.0);
    init_y_ = declare_parameter<double>("init_y", 0.0);
    init_z_ = declare_parameter<double>("init_z", 0.4);
    command_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "command", 10, std::bind(&PositionCommandToOdometry::commandCallback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ucon_sim/vehicle", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/ucon_sim/path", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&PositionCommandToOdometry::publishOdometry, this));
  }

private:
  void commandCallback(quadrotor_msgs::msg::PositionCommand::ConstSharedPtr command)
  {
    command_ = *command;
    received_command_ = true;
  }

  void publishOdometry()
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    if (!received_command_) {
      odom.pose.pose.position.x = init_x_;
      odom.pose.pose.position.y = init_y_;
      odom.pose.pose.position.z = init_z_;
      odom.pose.pose.orientation.w = 1.0;
      odom_pub_->publish(odom);
      publishMarker(odom);
      return;
    }

    odom.pose.pose.position = command_.position;
    Eigen::Vector3d acceleration(command_.acceleration.x, command_.acceleration.y, command_.acceleration.z + 9.8);
    Eigen::Vector3d heading(std::cos(command_.yaw), std::sin(command_.yaw), 0.0);
    Eigen::Vector3d heading_y(-std::sin(command_.yaw), std::cos(command_.yaw), 0.0);
    Eigen::Vector3d body_x = heading_y.cross(acceleration).normalized();
    Eigen::Vector3d body_y = acceleration.cross(body_x).normalized();
    Eigen::Vector3d body_z = body_x.cross(body_y);
    Eigen::Matrix3d rotation;
    rotation.col(0) = body_x;
    rotation.col(1) = body_y;
    rotation.col(2) = body_z;
    Eigen::Quaterniond orientation(rotation);
    odom.pose.pose.orientation.w = orientation.w();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.twist.twist.linear = command_.velocity;
    odom.twist.twist.angular = command_.acceleration;
    odom_pub_->publish(odom);
    publishMarker(odom);
    publishPath(odom);
  }

  void publishMarker(const nav_msgs::msg::Odometry &odom)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = odom.header;
    marker.ns = "uav_simulator";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = odom.pose.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.35;
    marker.color.r = 0.1;
    marker.color.g = 0.7;
    marker.color.b = 0.9;
    marker.color.a = 1.0;
    marker.mesh_resource = "package://poscmd_2_odom/meshes/fake_drone.dae";
    marker.mesh_use_embedded_materials = true;
    marker_pub_->publish(marker);
  }

  void publishPath(const nav_msgs::msg::Odometry &odom)
  {
    if (++path_tick_ < 10)
      return;
    path_tick_ = 0;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path_.header = odom.header;
    path_.poses.push_back(pose);
    if (path_.poses.size() > 3000)
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 1000);
    path_pub_->publish(path_);
  }

  double init_x_{0.0}, init_y_{0.0}, init_z_{0.4};
  bool received_command_{false};
  quadrotor_msgs::msg::PositionCommand command_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path_;
  int path_tick_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionCommandToOdometry>());
  rclcpp::shutdown();
  return 0;
}
