#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <traj_utils/msg/poly_traj.hpp>

class TrajectoryServer : public rclcpp::Node
{
public:
  TrajectoryServer() : Node("traj_server")
  {
    trajectory_sub_ = create_subscription<traj_utils::msg::PolyTraj>(
      "planning/trajectory", 10,
      std::bind(&TrajectoryServer::trajectoryCallback, this, std::placeholders::_1));
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
      "planning/heartbeat", 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) { last_heartbeat_ = now(); });
    command_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 50);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&TrajectoryServer::publishCommand, this));
    RCLCPP_INFO(get_logger(), "ROS 2 trajectory server ready");
  }

private:
  static double evaluate(const std::vector<float> &coefficients, size_t offset, double t, int derivative)
  {
    double value = 0.0;
    for (int index = 0; index < 6 - derivative; ++index)
    {
      int power = 5 - index;
      double factor = 1.0;
      for (int order = 0; order < derivative; ++order)
        factor *= power - order;
      value += factor * coefficients[offset + index] * std::pow(t, power - derivative);
    }
    return value;
  }

  void trajectoryCallback(traj_utils::msg::PolyTraj::ConstSharedPtr message)
  {
    const size_t expected = message->duration.size() * 6;
    if (message->order != 5 || message->duration.empty() ||
        message->coef_x.size() != expected || message->coef_y.size() != expected ||
        message->coef_z.size() != expected)
    {
      RCLCPP_ERROR(get_logger(), "Rejected malformed polynomial trajectory");
      return;
    }
    trajectory_ = *message;
    start_time_ = rclcpp::Time(message->start_time);
    has_trajectory_ = true;
    RCLCPP_INFO(get_logger(), "Executing trajectory %d with %zu pieces",
                message->traj_id, message->duration.size());
  }

  void publishCommand()
  {
    if (!has_trajectory_)
      return;
    if (last_heartbeat_.nanoseconds() != 0 && (now() - last_heartbeat_).seconds() > 0.5)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Planner heartbeat lost; holding last command");
      return;
    }

    double elapsed = std::max(0.0, (now() - start_time_).seconds());
    size_t piece = trajectory_.duration.size() - 1;
    double local_t = elapsed;
    for (size_t index = 0; index < trajectory_.duration.size(); ++index)
    {
      if (local_t <= trajectory_.duration[index])
      {
        piece = index;
        break;
      }
      local_t -= trajectory_.duration[index];
    }
    local_t = std::min(local_t, static_cast<double>(trajectory_.duration[piece]));
    const size_t offset = piece * 6;

    quadrotor_msgs::msg::PositionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "world";
    command.trajectory_id = trajectory_.traj_id;
    command.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    command.position.x = evaluate(trajectory_.coef_x, offset, local_t, 0);
    command.position.y = evaluate(trajectory_.coef_y, offset, local_t, 0);
    command.position.z = evaluate(trajectory_.coef_z, offset, local_t, 0);
    command.velocity.x = evaluate(trajectory_.coef_x, offset, local_t, 1);
    command.velocity.y = evaluate(trajectory_.coef_y, offset, local_t, 1);
    command.velocity.z = evaluate(trajectory_.coef_z, offset, local_t, 1);
    command.acceleration.x = evaluate(trajectory_.coef_x, offset, local_t, 2);
    command.acceleration.y = evaluate(trajectory_.coef_y, offset, local_t, 2);
    command.acceleration.z = evaluate(trajectory_.coef_z, offset, local_t, 2);
    command.jerk.x = evaluate(trajectory_.coef_x, offset, local_t, 3);
    command.jerk.y = evaluate(trajectory_.coef_y, offset, local_t, 3);
    command.jerk.z = evaluate(trajectory_.coef_z, offset, local_t, 3);
    const double horizontal_speed = std::hypot(command.velocity.x, command.velocity.y);
    command.yaw = horizontal_speed > 1e-3 ? std::atan2(command.velocity.y, command.velocity.x) : last_yaw_;
    command.yaw_dot = 0.0;
    last_yaw_ = command.yaw;
    command_pub_->publish(command);
  }

  bool has_trajectory_{false};
  double last_yaw_{0.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heartbeat_{0, 0, RCL_ROS_TIME};
  traj_utils::msg::PolyTraj trajectory_;
  rclcpp::Subscription<traj_utils::msg::PolyTraj>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryServer>());
  rclcpp::shutdown();
  return 0;
}
