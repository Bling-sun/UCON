#ifndef _mot_mapping_H_
#define _mot_mapping_H_

#include <chrono>
#include <queue>
#include <map>
#include <set>
// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <obj_state_msgs/msg/objects_states.hpp>
#include <obj_state_msgs/msg/state.hpp>
// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
// Algorithm
#include "DBSCAN_kdtree.h"
#include "Hungarian.h"
#include "target_ekf.hpp"
#include "ikd_Tree.h"
#include "ikd_Tree_impl.h"

using namespace std;

// typedef pcl::PointXYZ PointType;
// typedef pcl::PointCloud<PointType> Points;
// typedef pcl::PointCloud<PointType>::Ptr PointsPtr;
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

struct MapParam
{
  // DBSCAN Params
  int core_pts;
  double tolerance;
  double floor;
  double ceiling;
  int min_cluster;
  int max_cluster;
  // Dynamic Detection Params
  double thresh_dist;
  double thresh_var;
  // Lidar Params
  Eigen::Vector3d Range;
  // ROS Params
  std::string odom_topic;
  std::string lidar_topic;
};

struct MapData
{
  Eigen::Matrix3d R; // Currnet IMU Orientation
  Eigen::Vector3d T; // Currnet IMU Positon
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ObjectState
{
  // int id;
  Eigen::Vector3d pos; 
  Eigen::Vector3d vel;
  pcl::PointXYZ min_pt;
  pcl::PointXYZ max_pt;
  PointsPtr tra_pts; 
  bool on_tracker = false;
  bool is_dynamic = false;
  // Eigen::Vector3d center;
  // Eigen::Vector3d size;
};

namespace mot_mapping
{

  class MappingRos
  {

  private:
    pcl::VoxelGrid<PointType> vox;
    DBSCANKdtreeCluster<PointType> dbscan;
    KD_TREE<PointType>::Ptr ikdtree_ptr;
    KD_TREE<PointType>::Ptr pre_ikdtree_ptr;

    PointsPtr cloud_world;
    int frame_num;
    std::deque<PointsPtr> previous_points;
    std::deque<PointsPtr> buffer;
    std::deque<PointsPtr> input_point;
    std::vector<Eigen::Vector3d> previous_p;
    std::vector<Eigen::Vector3d> previous_v;
    std::vector<pcl::PointIndices> deleted_indices;
    std::vector<BoxPointType> delete_boxes;

    std::vector<std::shared_ptr<Ekf>> trackers;
    std::vector<ObjectState> detections;

    rclcpp::Time t_start;
    rclcpp::Time cloud_stamp_{0, 0, RCL_ROS_TIME};
    double frame_dt_{0.02};
    double box_padding_{0.0};
    bool use_ground_truth_{false};
    double truth_range_{4.5};
    std::map<std::string, std::set<std::pair<std::string, int>>> marker_ids_;

  private:
    rclcpp::Node::SharedPtr nh;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPub, staticMapPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edgePub, clusterPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr idTextPub, objectPosePub, sectorPub;
    rclcpp::Publisher<obj_state_msgs::msg::ObjectsStates>::SharedPtr statesPub;
    int id;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<obj_state_msgs::msg::ObjectsStates>::SharedPtr truth_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;

    unique_ptr<MapData> md_;
    unique_ptr<MapParam> mp_;

    // float calc_dist(PointType p1, PointType p2);
    // void generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths);
    void pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w);
    double compute2DIoU(const pcl::PointXYZ& min1, const pcl::PointXYZ& max1, const pcl::PointXYZ& min2, const pcl::PointXYZ& max2);
    Eigen::Vector2d getBox2DSize(const pcl::PointXYZ& min_pt, const pcl::PointXYZ& max_pt);
    double box2DSizeDiff(const pcl::PointXYZ& min1, const pcl::PointXYZ& max1, const pcl::PointXYZ& min2, const pcl::PointXYZ& max2);
    void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void truthCallback(obj_state_msgs::msg::ObjectsStates::ConstSharedPtr msg);
    void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom);
    void cloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                           const nav_msgs::msg::Odometry::ConstSharedPtr &odom);
    void mapPubCallback();
    void visualizeFunction(const std::vector<int> tra_id);
    void visualizeDbscan();

    void publishStates(const obj_state_msgs::msg::ObjectsStates& states);
    void publishBoxes(visualization_msgs::msg::MarkerArray& boxes);
    void publishObjectPoses(visualization_msgs::msg::MarkerArray& poses);
    void publishIDTexts(visualization_msgs::msg::MarkerArray& idTexts);
    void publishSectors(visualization_msgs::msg::MarkerArray& markers);
    void publishMarkers(
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
        visualization_msgs::msg::MarkerArray& markers, const std::string& key);

  public:
    explicit MappingRos(rclcpp::Node::SharedPtr nh);
    ~MappingRos();

    void init();
  };

}

#endif
