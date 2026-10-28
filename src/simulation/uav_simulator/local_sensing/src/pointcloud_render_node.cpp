#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudRenderer : public rclcpp::Node
{
public:
  PointCloudRenderer() : Node("pcl_render_node")
  {
    sensing_horizon_ = declare_parameter<double>("sensing_horizon", 8.0);
    sensing_rate_ = declare_parameter<double>("sensing_rate", 30.0);
    resolution_ = declare_parameter<double>("resolution", 0.1);
    global_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_map", rclcpp::QoS(1).reliable(),
      std::bind(&PointCloudRenderer::globalMapCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS(),
      std::bind(&PointCloudRenderer::odometryCallback, this, std::placeholders::_1));
    dynamic_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "dynamic_map", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&PointCloudRenderer::dynamicMapCallback, this, std::placeholders::_1));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pcl_render_node/cloud", 1);
    static_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pcl_render_node/static_cloud", 1);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / sensing_rate_),
      std::bind(&PointCloudRenderer::render, this));
  }

private:
  void odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr odometry)
  {
    odometry_ = *odometry;
    has_odometry_ = true;
  }

  void globalMapCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (has_global_map_)
      return;

    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*message, input);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(resolution_, resolution_, resolution_);
    voxel_filter.setInputCloud(input.makeShared());
    voxel_filter.filter(global_map_);
    kdtree_.setInputCloud(global_map_.makeShared());
    has_global_map_ = true;
    RCLCPP_INFO(get_logger(), "Global point cloud received: %zu points", global_map_.size());
  }

  void dynamicMapCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    pcl::fromROSMsg(*message, dynamic_map_);
    dynamic_stamp_ = message->header.stamp;
    has_dynamic_map_ = true;
    render(); // Deliver new moving-obstacle samples without another timer wait.
  }

  void render()
  {
    if (!has_global_map_ || !has_odometry_)
      return;
    if (has_dynamic_map_ && has_rendered_dynamic_ && dynamic_stamp_ == rendered_stamp_)
      return;

    pcl::PointXYZ center(
      odometry_.pose.pose.position.x,
      odometry_.pose.pose.position.y,
      odometry_.pose.pose.position.z);
    std::vector<int> indices;
    std::vector<float> squared_distances;
    kdtree_.radiusSearch(center, sensing_horizon_, indices, squared_distances);

    pcl::PointCloud<pcl::PointXYZ> local_map;
    local_map.reserve(indices.size());
    for (const int index : indices) {
      const auto &point = global_map_.points[index];
      if (std::abs(point.z - center.z) / sensing_horizon_ > std::tan(M_PI / 6.0))
        continue;
      local_map.push_back(point);
    }
    sensor_msgs::msg::PointCloud2 static_output;
    pcl::toROSMsg(local_map, static_output);
    static_output.header.stamp = has_dynamic_map_
        ? dynamic_stamp_ : static_cast<builtin_interfaces::msg::Time>(now());
    static_output.header.frame_id = "world";
    static_cloud_pub_->publish(static_output);

    const Eigen::Vector3d center_vector(center.x, center.y, center.z);
    for (const auto &point : dynamic_map_) {
      const Eigen::Vector3d point_vector(point.x, point.y, point.z);
      if ((point_vector - center_vector).norm() <= sensing_horizon_)
        local_map.push_back(point);
    }
    local_map.width = local_map.size();
    local_map.height = 1;
    local_map.is_dense = true;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(local_map, output);
    output.header.stamp = has_dynamic_map_
        ? dynamic_stamp_ : static_cast<builtin_interfaces::msg::Time>(now());
    output.header.frame_id = "world";
    cloud_pub_->publish(output);
    if (has_dynamic_map_) {
      rendered_stamp_ = dynamic_stamp_;
      has_rendered_dynamic_ = true;
    }
  }

  double sensing_horizon_{8.0};
  double sensing_rate_{30.0};
  float resolution_{0.1F};
  bool has_global_map_{false};
  bool has_odometry_{false};
  bool has_dynamic_map_{false};
  bool has_rendered_dynamic_{false};
  builtin_interfaces::msg::Time dynamic_stamp_, rendered_stamp_;
  nav_msgs::msg::Odometry odometry_;
  pcl::PointCloud<pcl::PointXYZ> global_map_;
  pcl::PointCloud<pcl::PointXYZ> dynamic_map_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudRenderer>());
  rclcpp::shutdown();
  return 0;
}
