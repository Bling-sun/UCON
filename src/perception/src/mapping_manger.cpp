#include "mapping_manager.h"

namespace mot_mapping
{

  MappingRos::MappingRos(rclcpp::Node::SharedPtr nh) : nh(std::move(nh)) {}

  MappingRos::~MappingRos() { std::cout << "Exit Tracker" << std::endl; }

  void MappingRos::init()
  {
    t_start = nh->now();
    frame_num = 1;
    id = 0;

    cloud_world.reset(new Points);
    ikdtree_ptr.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));
    pre_ikdtree_ptr.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));

    mp_ = std::make_unique<MapParam>();
    md_ = std::make_unique<MapData>();

    mp_->core_pts = nh->declare_parameter<int>("dbscan.core_pts", 4);
    mp_->tolerance = nh->declare_parameter<double>("dbscan.tolerance", 0.02);
    mp_->min_cluster = nh->declare_parameter<int>("dbscan.min_cluster", 20);
    mp_->max_cluster = nh->declare_parameter<int>("dbscan.max_cluster", 400);
    mp_->thresh_dist = nh->declare_parameter<double>("detection.thresh_dist", 0.08);
    mp_->thresh_var = nh->declare_parameter<double>("detection.thresh_var", 0.28);
    mp_->floor = nh->declare_parameter<double>("filtering.floor", 0.04);
    mp_->ceiling = nh->declare_parameter<double>("filtering.ceiling", 0.6);
    mp_->Range[0] = nh->declare_parameter<double>("lidar.range_x", 15.0);
    mp_->Range[1] = nh->declare_parameter<double>("lidar.range_y", 15.0);
    mp_->Range[2] = nh->declare_parameter<double>("lidar.range_z", 5.0);
    mp_->odom_topic = nh->declare_parameter<std::string>("odom_topic", "/Odometry");
    mp_->lidar_topic = nh->declare_parameter<std::string>("lidar_topic", "/cloud_registered_body");
    box_padding_ = nh->declare_parameter<double>("visualization.box_padding", 0.0);
    use_ground_truth_ = nh->declare_parameter<bool>("use_ground_truth", false);
    truth_range_ = nh->declare_parameter<double>("truth_range", 4.5);
    const auto truth_topic = nh->declare_parameter<std::string>("truth_topic", "/map_generator/obj_truth");
    const auto static_topic = nh->declare_parameter<std::string>("static_lidar_topic", "/cloud_registered_static");

    // Publishers
    // dynamicPub = nh.advertise<sensor_msgs::msg::PointCloud2>("/dynamic_points", 10);
    mapPub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/map_ros", 10);
    staticMapPub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/static_map", 10);
    clusterPub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/clu_edge", 10);
    idTextPub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/object_id", 10);
    objectPosePub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/object_pose", 10);
    edgePub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/box_edge", 10);
    statesPub = nh->create_publisher<obj_state_msgs::msg::ObjectsStates>("/states", 10);
    sectorPub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/object_sector", 20);

    // Subscribers
    // Obstacle pointcloud + ego pose
    odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
        mp_->odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&MappingRos::odomCallback, this, std::placeholders::_1));
    if (use_ground_truth_)
    {
      truth_sub_ = nh->create_subscription<obj_state_msgs::msg::ObjectsStates>(
          truth_topic, rclcpp::QoS(1),
          std::bind(&MappingRos::truthCallback, this, std::placeholders::_1));
      cloud_sub_ = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
          static_topic, rclcpp::SensorDataQoS().keep_last(1),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            if (msg->header.frame_id != "world") return;
            mapPub->publish(*msg);
            staticMapPub->publish(*msg);
          });
      RCLCPP_INFO(nh->get_logger(), "Dynamic boxes/states use simulator ground truth; static mapping uses %s", static_topic.c_str());
      return;
    }
    cloud_sub_ = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        mp_->lidar_topic, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&MappingRos::cloudCallback, this, std::placeholders::_1));

    map_pub_timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&MappingRos::mapPubCallback, this));
  }

  void MappingRos::pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w)
  {
    p_w->points.reserve(p_b->points.size());
    for (const auto& pt_b : p_b->points)
    {
      double r2 = pt_b.x * pt_b.x + pt_b.y * pt_b.y;
      if (r2 < 0.053) 
        continue;

      Eigen::Vector3d p(pt_b.x, pt_b.y, pt_b.z);
      p = md_->R * p + md_->T;

      PointType pt_w;
      pt_w.x = p.x();
      pt_w.y = p.y();
      pt_w.z = p.z();
      p_w->points.push_back(pt_w);
    }
  }

  inline double MappingRos::compute2DIoU(
    const pcl::PointXYZ& min1, const pcl::PointXYZ& max1,
    const pcl::PointXYZ& min2, const pcl::PointXYZ& max2)
  {
      double ix_min = std::max(min1.x, min2.x);
      double iy_min = std::max(min1.y, min2.y);

      double ix_max = std::min(max1.x, max2.x);
      double iy_max = std::min(max1.y, max2.y);

      double inter_x = std::max(0.0, ix_max - ix_min);
      double inter_y = std::max(0.0, iy_max - iy_min);

      double inter_area = inter_x * inter_y;

      double area1 = (max1.x - min1.x) * (max1.y - min1.y);
      double area2 = (max2.x - min2.x) * (max2.y - min2.y);

      if (area1 <= 1e-6 || area2 <= 1e-6)
          return 0.0;

      double union_area = area1 + area2 - inter_area;
      return inter_area / union_area;
  }

  Eigen::Vector2d MappingRos::getBox2DSize(const pcl::PointXYZ& min_pt,
                                    const pcl::PointXYZ& max_pt)
  {
    return Eigen::Vector2d(
      max_pt.x - min_pt.x,
      max_pt.y - min_pt.y
    );
  }
  
  double MappingRos::box2DSizeDiff(
  const pcl::PointXYZ& min1, const pcl::PointXYZ& max1,
  const pcl::PointXYZ& min2, const pcl::PointXYZ& max2)
  {
    Eigen::Vector2d s1 = getBox2DSize(min1, max1);
    Eigen::Vector2d s2 = getBox2DSize(min2, max2);

    if (s1.norm() < 1e-3 || s2.norm() < 1e-3)
      return 1e9;

    return (s1 - s2).norm() / s1.norm();
  }

  void MappingRos::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    latest_odom_ = std::move(odom);
  }

  void MappingRos::truthCallback(obj_state_msgs::msg::ObjectsStates::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != "world" || !latest_odom_) return;
    const rclcpp::Time stamp(msg->header.stamp);
    if (cloud_stamp_.nanoseconds() != 0 && stamp <= cloud_stamp_) return;
    cloud_stamp_ = stamp;
    trackers.clear();
    std::vector<int> visible_ids;
    for (size_t index = 0; index < msg->states.size(); ++index)
    {
      const auto& state = msg->states[index];
      const Eigen::Vector3d center(state.position.x, state.position.y, state.position.z);
      const pcl::PointXYZ low(state.position.x - state.size.x / 2.0,
                              state.position.y - state.size.y / 2.0,
                              state.position.z - state.size.z / 2.0);
      const pcl::PointXYZ high(state.position.x + state.size.x / 2.0,
                               state.position.y + state.size.y / 2.0,
                               state.position.z + state.size.z / 2.0);
      const auto& ego = latest_odom_->pose.pose.position;
      // Intersect the complete obstacle AABB with the spherical sensing range.
      // Keep its full height/width when any part enters the range.
      const double dx = ego.x - std::clamp(ego.x, double(low.x), double(high.x));
      const double dy = ego.y - std::clamp(ego.y, double(low.y), double(high.y));
      const double dz = ego.z - std::clamp(ego.z, double(low.z), double(high.z));
      if (dx * dx + dy * dy + dz * dz > truth_range_ * truth_range_) continue;
      // Reuse the output adapter only: no clustering, association, or EKF step.
      auto tracker = std::make_shared<Ekf>(0.02);
      tracker->reset(center, PointsPtr(new Points), low, high, static_cast<int>(index));
      tracker->x.tail(3) << state.velocity.x, state.velocity.y, state.velocity.z;
      trackers.push_back(tracker);
      visible_ids.push_back(static_cast<int>(index));
    }
    visualizeFunction(visible_ids);
  }

  void MappingRos::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (latest_odom_)
      cloudOdomCallback(msg, latest_odom_);
  }

  void MappingRos::cloudOdomCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
  {
    const rclcpp::Time stamp =
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
            ? nh->now() : rclcpp::Time(msg->header.stamp);
    if (cloud_stamp_.nanoseconds() != 0)
    {
      if (stamp <= cloud_stamp_)
        return; // Do not estimate motion twice from the same sensor sample.
      frame_dt_ = std::clamp((stamp - cloud_stamp_).seconds(), 0.001, 0.5);
    }
    cloud_stamp_ = stamp;
    for (auto& tracker : trackers)
      tracker->predict(frame_dt_);
    // ================================ Point cloud preprocessing ================================
    PointsPtr raw_cloud(new Points);
    PointsPtr latest_cloud(new Points);
    pcl::fromROSMsg(*msg, *raw_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(mp_->floor, mp_->ceiling);
    pass.filter(*latest_cloud);

    // pass.setInputCloud(latest_cloud);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(0.15, 6);
    // pass.filter(*latest_cloud);

    // pass.setInputCloud(latest_cloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(0.15, 6);
    // pass.filter(*latest_cloud);


    md_->T << odom->pose.pose.position.x,
        odom->pose.pose.position.y,
        odom->pose.pose.position.z;

    md_->R = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                odom->pose.pose.orientation.y, odom->pose.pose.orientation.z)
                 .toRotationMatrix();

    // clear screen
    // printf("\033[2J");
    // printf("\033[1;1H");

    // Pointcloud Filter
    PointsPtr cloud_filtered(new Points);
    vox.setInputCloud(latest_cloud);
    vox.setLeafSize(0.05, 0.05, 0.05);
    vox.filter(*cloud_filtered);

    cloud_world.reset(new Points);
    PointsPtr PointToAdd(new Points);
    // The simulator publishes world-frame points. Applying the vehicle pose
    // again would make detected obstacles move with the vehicle.
    if (msg->header.frame_id == "world")
      *cloud_world = *cloud_filtered;
    else
      pointsBodyToWorld(cloud_filtered, cloud_world);
 
    
    sensor_msgs::msg::PointCloud2 map_ros;
    // PointsPtr OutputPoints(new Points);
    // vox.setInputCloud(cloud_world);
    // vox.setLeafSize(0.05, 0.05, 0.05);
    // vox.filter(*OutputPoints);

    pcl::toROSMsg(*cloud_world, map_ros);
    map_ros.header.stamp = msg->header.stamp;
    map_ros.header.frame_id = "world";
    mapPub->publish(map_ros);
    if (cloud_world->empty())
    {
      trackers.clear();
      detections.clear();
      previous_p.clear();
      previous_v.clear();
      previous_points.clear();
      frame_num = 1;
      visualizeFunction({});
      return;
    }

    
    // std::cout << "================================ Perception Start ================================"  << std::endl;
    // compTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    // std::cout << "Filter Time Cost (ms): " << compTime << std::endl;
    // tic = std::chrono::high_resolution_clock::now();
    // std::cout << "Input Size:" << cloud_world->size() << std::endl;

    // ================================ DBSCAN clustering ================================
    std::vector<pcl::PointIndices> cluster_indices;
    dbscan.setCorePointMinPts(mp_->core_pts);
    dbscan.setClusterTolerance(mp_->tolerance);
    dbscan.setMinClusterSize(mp_->min_cluster);
    dbscan.setMaxClusterSize(mp_->max_cluster);
    dbscan.setInputCloud(cloud_world);
    dbscan.setSearchMethod();
    dbscan.extractNano(cluster_indices);

    // compTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    // std::cout << "Cluster Time Cost (ms): " << compTime << std::endl;
    // tic = std::chrono::high_resolution_clock::now();
    // std::cout << "Cluster size: " << cluster_indices.size() << std::endl;

    // ================================ Build kd-tree ================================
    // First frame: only store pointcloud
    if (frame_num < 2)
    {
      previous_points.push_back(cloud_world);
      frame_num++;
      return;
    }
    int pt_size = previous_points[0]->points.size();
    // Build ikd-tree
    if (pre_ikdtree_ptr->Root_Node == nullptr)
    {
      pre_ikdtree_ptr->Build(previous_points[0]->points);
      previous_points.push_back(cloud_world); 
      previous_points.pop_front();
    }
    else
    {
      PointVector PointNoNeedDownsample;
      // bool need_add = true;

      for (int i = 0; i < pt_size; ++i)
      {
        PointType mid_point;
        mid_point.x = floor(previous_points[0]->points[i].x / 0.1) * 0.1 + 0.5 * 0.1;
        mid_point.y = floor(previous_points[0]->points[i].y / 0.1) * 0.1 + 0.5 * 0.1;
        mid_point.z = floor(previous_points[0]->points[i].z / 0.1) * 0.1 + 0.5 * 0.1;
        // float dist  = calc_dist(cloud_filtered->points[i],mid_point);
        PointVector points_near;
        vector<float> pointSearchSqDis(5);
        pre_ikdtree_ptr->Nearest_Search(mid_point, 1, points_near, pointSearchSqDis);

        if (fabs(points_near[0].x - mid_point.x) > 0.5 * 0.1 ||
            fabs(points_near[0].y - mid_point.y) > 0.5 * 0.1 ||
            fabs(points_near[0].z - mid_point.z) > 0.5 * 0.1)
        {
          PointNoNeedDownsample.push_back(previous_points[0]->points[i]);
          PointToAdd->points.push_back(previous_points[0]->points[i]);
        }
      }
      // int add_point_size = pre_ikdtree_ptr->Add_Points(PointToAdd, true);
      pre_ikdtree_ptr->Add_Points(PointNoNeedDownsample, false);
      input_point.push_back(PointToAdd);
      if (input_point.size() > 20)
      {
        PointVector PointDelete;
        for (auto &delet_pt : input_point[0]->points)
        {
          PointDelete.push_back(delet_pt);
        }
        pre_ikdtree_ptr->Delete_Points(PointDelete);
        input_point.pop_front();
      }

      previous_points.push_back(cloud_world);
      previous_points.pop_front();
    }
    ikdtree_ptr->Build(cloud_world->points);
    
    // compTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    // std::cout << "Two ikd-Tree Time Cost (ms): " << compTime << std::endl;
    // tic = std::chrono::high_resolution_clock::now();
    // std::cout << "pre-ikd-Tree Size: " << pre_ikdtree_ptr->size() << std::endl;
    // std::cout << "ikd-Tree Size: " << ikdtree_ptr->size() << std::endl;

    // =========================== Classify clusters as dynamic/static ===========================
    // PointsPtr dynamic_points(new Points);
    // Classify each cluster and store in detections
    detections.clear();
    deleted_indices.clear();
    // detections.reserve(cluster_indices.size());
    for (auto &getIndices : cluster_indices)
    {
      PointsPtr cluster(new Points);
      double avg_dist = 0;
      Eigen::Vector3d cluster_center(0, 0, 0);
      std::vector<double> avg_buffer;
      for (auto &index : getIndices.indices)
      {
        cluster->points.push_back(cloud_world->points[index]);
        PointVector points_near;
        vector<float> pointSearchSqDis(5);
        pre_ikdtree_ptr->Nearest_Search(cloud_world->points[index], 3, points_near, pointSearchSqDis);
        avg_dist += sqrt(pointSearchSqDis[0]);
        avg_buffer.push_back(sqrt(pointSearchSqDis[0]));
        cluster_center += Eigen::Vector3d(cloud_world->points[index].x, cloud_world->points[index].y, cloud_world->points[index].z);
      }

      pcl::PointXYZ minPt, maxPt;
      pcl::getMinMax3D(*cluster, minPt, maxPt);
      float dx = maxPt.x - minPt.x;  
      float dy = maxPt.y - minPt.y;  
      if (dx > 1.2f || dy > 1.2f) continue;

      avg_dist = avg_dist / getIndices.indices.size();
      cluster_center = cluster_center / getIndices.indices.size();
      double var_dist = 0;
      for (auto &dist : avg_buffer)
      {
        var_dist += (dist - avg_dist) * (dist - avg_dist) / (avg_dist * avg_dist);
      }
      var_dist = var_dist / getIndices.indices.size();

      ObjectState obj;
      obj.pos = cluster_center;
      obj.vel = Eigen::Vector3d::Zero();  
      obj.tra_pts = cluster;
      obj.min_pt = minPt;
      obj.max_pt = maxPt;
      if (avg_dist > mp_->thresh_dist && avg_dist < 5.0 && var_dist < mp_->thresh_var)
      {
        obj.is_dynamic = true; 
      }
      detections.push_back(obj);  
    }
    visualizeDbscan();

    // compTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    // std::cout << "Judging Dynamic and Static Time Cost (ms): " << compTime << std::endl;
    // tic = std::chrono::high_resolution_clock::now();
    // std::cout << "detections Size: " << detections.size() << std::endl;
   
    // =========================== Initialize trackers on first frame ===========================
    if(trackers.size() == 0 && detections.size() > 0) {
      std::vector<int> initial_ids;
      for(size_t i = 0; i < detections.size(); ++i) {
        if (detections[i].is_dynamic == true)
        {
          std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
          ekfPtr->reset(detections[i].pos, detections[i].tra_pts, detections[i].min_pt, detections[i].max_pt, id);
          trackers.push_back(ekfPtr);
          initial_ids.push_back(ekfPtr->id);
          id++;
          detections[i].on_tracker = true;
          previous_p.push_back(detections[i].pos);
          Eigen::Vector3d v0(0,0,0);
          previous_v.push_back(v0);
        }
      }
      visualizeFunction(initial_ids);
      return;
    }

    // =========================== Data association with previous trackers ===========================
    // Three cases: strong match / weak match / delete tracker
    
    std::vector<int> delete_tracker;
    std::vector<int> tracker_vis;
  
    for (size_t i = 0; i < trackers.size(); ++i)
    {
      double min_dist = 1000;
      int min_idx = -1;
      for (size_t j = 0; j < detections.size(); ++j)
      {
        if (!detections[j].on_tracker)
        {
          double pos_dist = (trackers[i]->pos() - detections[j].pos).norm();
          if(pos_dist < min_dist){
            min_dist = pos_dist;
            min_idx = j;
          }
        }
      }
      // double iou = compute2DIoU(trackers[i]->min_pt, trackers[i]->max_pt,
      //   detections[min_idx].min_pt, detections[min_idx].max_pt);
      
      double size_diff = min_idx >= 0
          ? box2DSizeDiff(trackers[i]->min_pt, trackers[i]->max_pt,
                         detections[min_idx].min_pt, detections[min_idx].max_pt)
          : std::numeric_limits<double>::infinity();
      // Case 1: strong match
      
      if (min_idx >= 0 && min_dist < 0.6 && size_diff < 0.4)
      {
        // std::cout << "Strong Association: " << trackers[i]->id <<std::endl;
        Eigen::Vector3d vel_detect =
            (detections[min_idx].pos - trackers[i]->measurement_position) / frame_dt_;
        trackers[i]->update(detections[min_idx].pos, vel_detect, detections[min_idx].min_pt, detections[min_idx].max_pt, detections[min_idx].tra_pts);
        // trackers[i]->cum = trackers[i]->cum + 1;
          
        detections[min_idx].on_tracker = true; 
        tracker_vis.push_back(trackers[i]->id);
        
      // Case 2: weak match
      }else if(ikdtree_ptr->Root_Node != nullptr)
      {
        // std::cout << "Weak Association: " << trackers[i]->id << std::endl;
        // std::cout << "min_dist: " << min_dist <<  "size_diff:" << size_diff << std::endl;
        if (min_dist < 1 && min_idx >= 0)
        {
          detections[min_idx].on_tracker = true; 
        }
  
        PointsPtr tracker_pts(new Points);
        Eigen::Vector3d tracker_pos(0, 0, 0);
        int sz = 0;
        for (const auto& pt : trackers[i]->tra_pts->points)
        {
          PointVector point_near;
          vector<float> pointSearchSqDis(1);
          pcl::PointXYZ predicted_point = pt;
          const Eigen::Vector3d displacement = trackers[i]->vel() * frame_dt_;
          predicted_point.x += displacement.x();
          predicted_point.y += displacement.y();
          predicted_point.z += displacement.z();
          ikdtree_ptr->Nearest_Search(predicted_point, 1, point_near, pointSearchSqDis);
          // Eigen::Vector3d point_eigen(point_near[0].x, point_near[0].y, point_near[0].z);
          // float dis = (point_eigen - trackers[i]->pos()).norm();
          if (!point_near.empty() && sqrt(pointSearchSqDis[0]) < 0.40)
          {
            
            sz++;
            tracker_pts->points.push_back(point_near[0]);
            tracker_pos += Eigen::Vector3d(point_near[0].x, point_near[0].y, point_near[0].z);
          }
        }
        if(sz != 0 && static_cast<double>(tracker_pts->points.size()) / trackers[i]->tra_pts->points.size() > 0.4){
          tracker_pos = tracker_pos / sz;
          pcl::PointXYZ minPt, maxPt;
          pcl::getMinMax3D(*tracker_pts, minPt, maxPt);
          Eigen::Vector3d vel_detect =
              (tracker_pos - trackers[i]->measurement_position) / frame_dt_;
          // Follow the current observation, including its point template.
          trackers[i]->update(tracker_pos, vel_detect, minPt, maxPt, tracker_pts);
          // trackers[i]->cum = trackers[i]->cum + 1;
          tracker_vis.push_back(trackers[i]->id);
        // Case 3: delete
        }else
        {
          // std::cout << "DELETE : " << i << std::endl;
          delete_tracker.push_back(i);
        }
      }
    }

    std::sort(delete_tracker.begin(), delete_tracker.end(), std::greater<int>());
    for (const auto& idx : delete_tracker) {
        if (idx >= 0 && static_cast<size_t>(idx) < trackers.size()) {
            trackers.erase(trackers.begin() + idx);
        }
    }
    

    for (size_t i = 0; i < detections.size(); ++i)
    {
      if (!detections[i].is_dynamic || detections[i].on_tracker)
        continue;
      
      double min_dist = 1000;
      for (size_t j = 0; j < trackers.size(); ++j)
      {
        double pos_dist = (trackers[j]->pos() - detections[i].pos).norm();
        if(pos_dist < min_dist){
          min_dist = pos_dist;
        }
      }
      if (min_dist > 0.8)
      {
        std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
        ekfPtr->reset(detections[i].pos, detections[i].tra_pts, detections[i].min_pt, detections[i].max_pt, id);
        id++;
        trackers.push_back(ekfPtr);
        detections[i].on_tracker = true;
        tracker_vis.push_back(ekfPtr->id);
      }
    }

    previous_p.clear();
    previous_v.clear();
    for (size_t i = 0; i < trackers.size(); ++i ) {
      previous_p.push_back(trackers[i]->pos());
      previous_v.push_back(trackers[i]->vel());
    }
    // compTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    // std::cout << "Track Time Cost (ms): " << compTime << std::endl;
    // tic = std::chrono::high_resolution_clock::now();
    // std::cout << "================================ Perception End ================================"  << std::endl;
    
    
    // std::cout << "Trackers Size: " << trackers.size() << std::endl;
    // std::cout << "Detections Size: " << detections.size() << std::endl;
    
    visualizeFunction(tracker_vis);
  }

  void MappingRos::mapPubCallback() {
    if (!cloud_world)
      return;
    PointsPtr static_cloud(new Points);
    static_cloud->points.reserve(cloud_world->points.size());
    const double box_margin = 0.1;

    for (const auto& pt : cloud_world->points)
    {
      bool is_dynamic = false;

      for (const auto& tracker : trackers)
      {
        // if (tracker->age < 3) continue;

        // Remove the entire observed object, not just a slice around its center.
        if (pt.x >= tracker->min_pt.x - box_margin &&
            pt.x <= tracker->max_pt.x + box_margin &&
            pt.y >= tracker->min_pt.y - box_margin &&
            pt.y <= tracker->max_pt.y + box_margin &&
            pt.z >= tracker->min_pt.z - box_margin &&
            pt.z <= tracker->max_pt.z + box_margin)
        {
          is_dynamic = true;
          break;
        }
      }

      if (!is_dynamic)
        static_cloud->points.push_back(pt);
    }

    sensor_msgs::msg::PointCloud2 static_msg;
    pcl::toROSMsg(*static_cloud, static_msg);
    static_msg.header.frame_id = "world";
    static_msg.header.stamp = nh->now();
    staticMapPub->publish(static_msg);
  }

  void MappingRos::visualizeDbscan() {
    visualization_msgs::msg::MarkerArray boxes;
    
    for (size_t i = 0; i < detections.size(); ++i) {
      float center_x = (detections[i].min_pt.x + detections[i].max_pt.x) / 2.0;
      float center_y = (detections[i].min_pt.y + detections[i].max_pt.y) / 2.0;
      float center_z = (detections[i].min_pt.z + detections[i].max_pt.z) / 2.0;

      float size_x = detections[i].max_pt.x - detections[i].min_pt.x;
      float size_y = detections[i].max_pt.y - detections[i].min_pt.y;
      float size_z = detections[i].max_pt.z - detections[i].min_pt.z;

      visualization_msgs::msg::Marker cubeMarker;
      cubeMarker.id = i;
      cubeMarker.header.stamp = cloud_stamp_;
      cubeMarker.header.frame_id = "world";
      cubeMarker.ns = "dbscan_cubes";

      cubeMarker.pose.position.x = center_x;
      cubeMarker.pose.position.y = center_y;
      cubeMarker.pose.position.z = center_z;
      
      cubeMarker.pose.orientation.x = 0.0;
      cubeMarker.pose.orientation.y = 0.0;
      cubeMarker.pose.orientation.z = 0.0;
      cubeMarker.pose.orientation.w = 1.0;
   
      cubeMarker.type = visualization_msgs::msg::Marker::CUBE;
      cubeMarker.action = visualization_msgs::msg::Marker::ADD;

      cubeMarker.scale.x = size_x;
      cubeMarker.scale.y = size_y;
      cubeMarker.scale.z = size_z;
      
      cubeMarker.color.r = 0.6;  
      cubeMarker.color.g = 0.8;  
      cubeMarker.color.b = 1.0;  
      cubeMarker.color.a = 0.3;

      cubeMarker.lifetime = rclcpp::Duration::from_seconds(0.5);
      boxes.markers.push_back(cubeMarker);
    }
    clusterPub->publish(boxes);
  }

  void MappingRos::visualizeFunction(const std::vector<int> tra_id){
    visualization_msgs::msg::MarkerArray idTexts;
    visualization_msgs::msg::MarkerArray poses;
    visualization_msgs::msg::MarkerArray boxes;
    visualization_msgs::msg::MarkerArray sectorArray;
    obj_state_msgs::msg::ObjectsStates states;
    states.header.stamp = cloud_stamp_;
    states.header.frame_id = "world";
    
    for (const int& idx : tra_id) {
      for (size_t i = 0; i < trackers.size(); ++i)
      {
        if (idx == trackers[i]->id)
        {
          // std::cout <<  "trackers[i]->tra_pts->size(): " << trackers[i]->tra_pts->size() << std::endl;
          visualization_msgs::msg::Marker edgeMarker;
          edgeMarker.id = trackers[i]->id;
          edgeMarker.header.stamp = cloud_stamp_;
          edgeMarker.header.frame_id = "world";
          edgeMarker.pose.orientation.w = 1.00;
          edgeMarker.lifetime = rclcpp::Duration::from_seconds(0.5);
          edgeMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          edgeMarker.action = visualization_msgs::msg::Marker::ADD;
          edgeMarker.ns = "edge";
          edgeMarker.color.r = 0.00;
          edgeMarker.color.g = 1.00;
          edgeMarker.color.b = 0.00;
          edgeMarker.color.a = 0.80;
          edgeMarker.scale.x = 0.03;
          pcl::PointXYZ box_min = trackers[i]->min_pt;
          pcl::PointXYZ box_max = trackers[i]->max_pt;
          box_min.x -= box_padding_; box_min.y -= box_padding_; box_min.z -= box_padding_;
          box_max.x += box_padding_; box_max.y += box_padding_; box_max.z += box_padding_;
          geometry_msgs::msg::Point point[8];
          point[0].x = box_min.x; point[0].y = box_max.y; point[0].z = box_max.z;
          point[1].x = box_min.x; point[1].y = box_min.y; point[1].z = box_max.z;
          point[2].x = box_min.x; point[2].y = box_min.y; point[2].z = box_min.z;
          point[3].x = box_min.x; point[3].y = box_max.y; point[3].z = box_min.z;
          point[4].x = box_max.x; point[4].y = box_max.y; point[4].z = box_min.z;
          point[5].x = box_max.x; point[5].y = box_min.y; point[5].z = box_min.z;
          point[6].x = box_max.x; point[6].y = box_min.y; point[6].z = box_max.z;
          point[7].x = box_max.x; point[7].y = box_max.y; point[7].z = box_max.z;
          for (int l = 0; l < 8; l++) {
            edgeMarker.points.push_back(point[l]);
          }
          edgeMarker.points.push_back(point[0]);
          edgeMarker.points.push_back(point[3]);
          edgeMarker.points.push_back(point[2]);
          edgeMarker.points.push_back(point[5]);
          edgeMarker.points.push_back(point[6]);
          edgeMarker.points.push_back(point[1]);
          edgeMarker.points.push_back(point[0]);
          edgeMarker.points.push_back(point[7]);
          edgeMarker.points.push_back(point[4]);
          boxes.markers.push_back(edgeMarker); 

          visualization_msgs::msg::Marker poseMarker;
          ObjectState state;
          state.pos = trackers[i]->pos();
          state.pos[2] = 0.4; // FIX2D
          state.vel = trackers[i]->vel();
          state.vel[2] = 0; // FIX2D
          poseMarker.id = 1000 + trackers[i]->id;
          poseMarker.header.stamp = cloud_stamp_;
          poseMarker.header.frame_id = "world";
          poseMarker.lifetime = rclcpp::Duration::from_seconds(0.5);
          poseMarker.type = visualization_msgs::msg::Marker::ARROW;
          poseMarker.action = visualization_msgs::msg::Marker::ADD;
          poseMarker.ns = "objectpose";
          poseMarker.color.r = 0.00;
          poseMarker.color.g = 1.00;
          poseMarker.color.b = 0.00;
          poseMarker.color.a = 1.00;
          poseMarker.scale.x = 0.10;
          poseMarker.scale.y = 0.18;
          poseMarker.scale.z = 0.30;
          poseMarker.pose.orientation.w = 1.0;
          geometry_msgs::msg::Point arrow[2];
          arrow[0].x = state.pos[0]; arrow[0].y = state.pos[1]; arrow[0].z = state.pos[2];
          arrow[1].x = state.pos[0] + 1.0*state.vel[0]/state.vel.norm(); 
          arrow[1].y = state.pos[1] + 1.0*state.vel[1]/state.vel.norm(); 
          arrow[1].z = state.pos[2] + 1.0*state.vel[2]/state.vel.norm();

          poseMarker.points.push_back(arrow[0]);
          poseMarker.points.push_back(arrow[1]);
          if (state.vel.norm() > 0.01)
            poses.markers.push_back(poseMarker);
          
          obj_state_msgs::msg::State statemsg;
          statemsg.header.stamp = cloud_stamp_;
          statemsg.position.x = state.pos[0]; statemsg.position.y = state.pos[1]; statemsg.position.z = state.pos[2];
          statemsg.velocity.x = state.vel[0]; statemsg.velocity.y = state.vel[1]; statemsg.velocity.z = state.vel[2];
          statemsg.header.frame_id = "world";
          statemsg.size.x = box_max.x - box_min.x;
          statemsg.size.y = box_max.y - box_min.y;
          statemsg.size.z = box_max.z - box_min.z;
          Eigen::Matrix3d Sigma_p = trackers[i]->Sigma.block<3,3>(0,0);
          statemsg.sigma[0] = Sigma_p(0, 0);
          statemsg.sigma[1] = Sigma_p(0, 1);
          statemsg.sigma[2] = Sigma_p(0, 2);
          statemsg.sigma[3] = Sigma_p(1, 0);
          statemsg.sigma[4] = Sigma_p(1, 1);
          statemsg.sigma[5] = Sigma_p(1, 2);
          statemsg.sigma[6] = Sigma_p(2, 0);
          statemsg.sigma[7] = Sigma_p(2, 1);
          statemsg.sigma[8] = Sigma_p(2, 2);
          states.states.push_back(statemsg);

          // Sector danger zone
          visualization_msgs::msg::Marker sectorMarker;
          sectorMarker.id = 2000 + trackers[i]->id;
          sectorMarker.header.stamp = cloud_stamp_;
          sectorMarker.header.frame_id = "world";
          sectorMarker.lifetime = rclcpp::Duration::from_seconds(0.5);
          sectorMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          sectorMarker.action = visualization_msgs::msg::Marker::ADD;
          sectorMarker.ns = "sector";
          sectorMarker.color.r = 1.0;
          sectorMarker.color.g = 0.0;
          sectorMarker.color.b = 0.0;
          sectorMarker.color.a = 0.8;
          sectorMarker.scale.x = 0.07;

          Eigen::Vector3d p = state.pos;
          Eigen::Vector3d v = state.vel;
          double vnorm = v.norm();
          Eigen::Vector3d v_hat = (vnorm > 1e-3 ? v / vnorm : Eigen::Vector3d(1,0,0));

          //--------------- Extract Covariance ---------------
          // Eigen::Matrix3d Sigma_p = trackers[i]->Sigma.block<3,3>(0,0);

          //--------------- Covariance projection ------------
          double sigma_parallel = v_hat.transpose() * Sigma_p * v_hat;

          // find orthogonal basis (u1,u2)
          Eigen::Vector3d u1;
          if (fabs(v_hat.x()) < 0.9) u1 = Eigen::Vector3d(1,0,0);
          else                       u1 = Eigen::Vector3d(0,1,0);
          u1 = (u1 - u1.dot(v_hat)*v_hat).normalized();
          double sigma_perp1 = u1.transpose() * Sigma_p * u1;

          Eigen::Vector3d u2 = v_hat.cross(u1).normalized();
          double sigma_perp2 = u2.transpose() * Sigma_p * u2;

          double sigma_perp = std::max(sigma_perp1, sigma_perp2);

          //--------------- Sector parameters ----------------
          double H = 0.02;
          double k_r = 1.0;
          double k_t = 0.01;
          double r_obj = 0.3;
          double margin = 0.2;

          double R = vnorm*H + k_r * sqrt(sigma_parallel + 1e-6) + r_obj + margin;
          double theta = atan( k_t * sqrt(sigma_perp + 1e-6) / (vnorm*H + 1e-3) );

          //--------------- Draw sector fan -----------------
          int N = 40;    // number of points on arc
          double start_angle = -theta;
          double end_angle   = theta;

          // orth basis: u1 direction = right boundary when angle=0
          // rotation on tangent plane: v_hat is forward, u1 is "right"
          for (int k = 0; k <= N; ++k) {
              double a = start_angle + (end_angle - start_angle) * k / N;

              // direction = cos(a)*v_hat + sin(a)*u1   (2D on tangent plane)
              Eigen::Vector3d dir = (v_hat * cos(a) + u1 * sin(a)).normalized();
              Eigen::Vector3d p_arc = p + R * dir;

              geometry_msgs::msg::Point q;
              q.x = p_arc.x();
              q.y = p_arc.y();
              q.z = p_arc.z();
              sectorMarker.points.push_back(q);
          }

    
          geometry_msgs::msg::Point p0;
          p0.x = p.x(); p0.y = p.y(); p0.z = p.z();
          sectorMarker.points.insert(sectorMarker.points.begin(), p0);
          sectorMarker.points.push_back(p0);

          sectorArray.markers.push_back(sectorMarker);
          
          // ================= ID TEXT MARKER =================
          visualization_msgs::msg::Marker textMarker;
          textMarker.header.frame_id = "world";
          textMarker.header.stamp = cloud_stamp_;
          textMarker.ns = "object_id";
          textMarker.id = 3000 + trackers[i]->id; 
          textMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          textMarker.action = visualization_msgs::msg::Marker::ADD;
          textMarker.lifetime = rclcpp::Duration::from_seconds(0.5);

          textMarker.pose.position.x = state.pos[0];
          textMarker.pose.position.y = state.pos[1];
          textMarker.pose.position.z = state.pos[2] + 0.6;  
          textMarker.pose.orientation.w = 1.0;

          textMarker.text = "ID: " + std::to_string(trackers[i]->id);

          textMarker.scale.z = 0.35;

          textMarker.color.r = 0.0;
          textMarker.color.g = 0.0;
          textMarker.color.b = 0.0;
          textMarker.color.a = 1.0;

          idTexts.markers.push_back(textMarker);
        } 
      }
    }
    publishStates(states);
    publishBoxes(boxes);
    publishObjectPoses(poses);
    publishIDTexts(idTexts);
    publishSectors(sectorArray);
  }

  inline void MappingRos::publishStates(
    const obj_state_msgs::msg::ObjectsStates& states)
    {
      // Empty updates are significant: they clear objects that disappeared
      // from the current perception frame in downstream planners.
      statesPub->publish(states);
    }

    void MappingRos::publishMarkers(
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
        visualization_msgs::msg::MarkerArray& markers, const std::string& key)
    {
      std::set<std::pair<std::string, int>> current;
      for (const auto& marker : markers.markers)
        current.emplace(marker.ns, marker.id);
      for (const auto& old : marker_ids_[key])
      {
        if (current.count(old))
          continue;
        visualization_msgs::msg::Marker removed;
        removed.header.frame_id = "world";
        removed.header.stamp = cloud_stamp_;
        removed.ns = old.first;
        removed.id = old.second;
        removed.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(removed);
      }
      marker_ids_[key] = std::move(current);
      publisher->publish(markers);
    }

    inline void MappingRos::publishBoxes(visualization_msgs::msg::MarkerArray& boxes)
    {
      publishMarkers(edgePub, boxes, "boxes");
    }

    inline void MappingRos::publishObjectPoses(visualization_msgs::msg::MarkerArray& poses)
    {
      publishMarkers(objectPosePub, poses, "poses");
    }

    inline void MappingRos::publishIDTexts(visualization_msgs::msg::MarkerArray& idTexts)
    {
      publishMarkers(idTextPub, idTexts, "ids");
    }

    inline void MappingRos::publishSectors(visualization_msgs::msg::MarkerArray& sectors)
    {
      publishMarkers(sectorPub, sectors, "sectors");
    }
}
