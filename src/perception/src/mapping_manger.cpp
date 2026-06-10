#include "mapping_manager.h"

namespace mot_mapping
{

  MappingRos::MappingRos(ros::NodeHandle &nh) : nh(nh) {}

  MappingRos::~MappingRos() { std::cout << "Exit Tracker" << std::endl; }

  void MappingRos::init()
  {
    t_start = ros::Time::now();
    frame_num = 1;
    id = 0;

    cloud_world.reset(new Points);
    ikdtree_ptr.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));
    pre_ikdtree_ptr.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));

    mp_ = std::make_unique<MapParam>();
    md_ = std::make_unique<MapData>();

    nh.param("dbscan/core_pts", mp_->core_pts, 4);
    nh.param("dbscan/tolerance", mp_->tolerance, 0.02);
    nh.param("dbscan/min_cluster", mp_->min_cluster, 20);
    nh.param("dbscan/max_cluster", mp_->max_cluster, 400);
    nh.param("detection/thresh_dist", mp_->thresh_dist, 0.08);
    nh.param("detection/thresh_var", mp_->thresh_var, 0.28);
    nh.param("filtering/floor", mp_->floor, 0.04);
    nh.param("filtering/ceiling", mp_->ceiling, 0.6);

    nh.param("lidar/range_x", mp_->Range[0], 15.0);
    nh.param("lidar/range_y", mp_->Range[1], 15.0);
    nh.param("lidar/range_z", mp_->Range[2], 5.0);
    nh.param("ros/odom_topic", mp_->odom_topic, string("/drone_0_visual_slam/odom"));
    nh.param("ros/lidar_topic", mp_->lidar_topic, string("/map_generator/obj_cloud"));

    // Publishers
    // dynamicPub = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_points", 10);
    mapPub = nh.advertise<sensor_msgs::PointCloud2>("/map_ros", 10);
    staticMapPub = nh.advertise<sensor_msgs::PointCloud2>("/static_map", 10);

    clusterPub = nh.advertise<visualization_msgs::MarkerArray>("/clu_edge", 10);
    idTextPub = nh.advertise<visualization_msgs::MarkerArray>("/object_id", 10);
    objectPosePub = nh.advertise<visualization_msgs::MarkerArray>("/object_pose", 10);
    edgePub = nh.advertise<visualization_msgs::MarkerArray>("/box_edge", 10);
    statesPub = nh.advertise<obj_state_msgs::ObjectsStates>("/states", 10);
    sectorPub = nh.advertise<visualization_msgs::MarkerArray>("/object_sector", 20);

    // Subscribers
    // Obstacle pointcloud + ego pose
    cloud_sub_.reset(
        new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, mp_->lidar_topic, 50));
    odom_sub_.reset(
        new message_filters::Subscriber<nav_msgs::Odometry>(nh, mp_->odom_topic, 25));
    sync_cloud_odom_.reset(new message_filters::Synchronizer<MappingRos::SyncPolicyCloudOdom>(
        MappingRos::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&MappingRos::cloudOdomCallback, this, _1, _2));

    ekf_predict_timer_ = nh.createTimer(ros::Duration(0.02), &MappingRos::ekfPredictCallback, this);
    map_pub_timer_ = nh.createTimer(ros::Duration(0.05), &MappingRos::mapPubCallback, this);
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

  void MappingRos::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                                     const nav_msgs::OdometryConstPtr &odom)
  {
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

    double compTime;
    std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
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
    pointsBodyToWorld(cloud_filtered, cloud_world);
 
    
    sensor_msgs::PointCloud2 map_ros;
    // PointsPtr OutputPoints(new Points);
    // vox.setInputCloud(cloud_world);
    // vox.setLeafSize(0.05, 0.05, 0.05);
    // vox.filter(*OutputPoints);

    pcl::toROSMsg(*cloud_world, map_ros);
    map_ros.header.frame_id = "world";
    mapPub.publish(map_ros);

    
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
      for(size_t i = 0; i < detections.size(); ++i) {
        if (detections[i].is_dynamic == true)
        {
          std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
          ekfPtr->reset(detections[i].pos, detections[i].tra_pts, detections[i].min_pt, detections[i].max_pt, id);
          trackers.push_back(ekfPtr);
          id++;
          detections[i].on_tracker = true;
          previous_p.push_back(detections[i].pos);
          Eigen::Vector3d v0(0,0,0);
          previous_v.push_back(v0);
        }
      }
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
      
      double size_diff = box2DSizeDiff(trackers[i]->min_pt, trackers[i]->max_pt,
        detections[min_idx].min_pt, detections[min_idx].max_pt);
      // Case 1: strong match
      
      if (min_idx >= 0 && min_dist < 0.6 && size_diff < 0.4)
      {
        // std::cout << "Strong Association: " << trackers[i]->id <<std::endl;
        Eigen::Vector3d vel_detect = (detections[min_idx].pos - previous_p[i]) / 0.02;
        trackers[i]->update(detections[min_idx].pos, 0.5*vel_detect + 0.5*previous_v[i], detections[min_idx].min_pt, detections[min_idx].max_pt, detections[min_idx].tra_pts);
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
          ikdtree_ptr->Nearest_Search(pt, 1, point_near, pointSearchSqDis);
          // Eigen::Vector3d point_eigen(point_near[0].x, point_near[0].y, point_near[0].z);
          // float dis = (point_eigen - trackers[i]->pos()).norm();
          if (sqrt(pointSearchSqDis[0]) < 0.40)
          {
            
            sz++;
            tracker_pts->points.push_back(point_near[0]);
            tracker_pos += Eigen::Vector3d(point_near[0].x, point_near[0].y, point_near[0].z);
          }
        }
        if(sz != 0 && tracker_pts->points.size() / trackers[i]->tra_pts->points.size() > 0.4){
          tracker_pos = tracker_pos / sz;
          pcl::PointXYZ minPt, maxPt;
          pcl::getMinMax3D(*tracker_pts, minPt, maxPt);
          Eigen::Vector3d vel_detect = (tracker_pos - previous_p[i]) / 0.02;
          // trackers[i]->update(tracker_pos, 0.5*vel_detect + 0.5*previous_v[i], minPt, maxPt, tracker_pts);
          pcl::PointXYZ weighted_min_pt;
          weighted_min_pt.x = 0.8 * trackers[i]->min_pt.x + 0.2 * minPt.x;
          weighted_min_pt.y = 0.8 * trackers[i]->min_pt.y + 0.2 * minPt.y;
          weighted_min_pt.z = 0.8 * trackers[i]->min_pt.z + 0.2 * minPt.z;

          // Compute weighted max_pt
          pcl::PointXYZ weighted_max_pt;
          weighted_max_pt.x = 0.8 * trackers[i]->max_pt.x + 0.2 * maxPt.x;
          weighted_max_pt.y = 0.8 * trackers[i]->max_pt.y + 0.2 * maxPt.y;
          weighted_max_pt.z = 0.8 * trackers[i]->max_pt.z + 0.2 * maxPt.z;
          trackers[i]->update(tracker_pos, 0.5*vel_detect + 0.5*previous_v[i], 
            weighted_min_pt, weighted_max_pt, trackers[i]->tra_pts);
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
        if (idx >= 0 && idx < trackers.size()) {
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

  void MappingRos::ekfPredictCallback(const ros::TimerEvent &e)
  {
    if (trackers.size() == 0)
      return;

    for (size_t i = 0; i < trackers.size(); ++i)
    {
      // double update_dt = (ros::Time::now() - trackers[i]->last_update_stamp_).toSec();
      trackers[i]->predict();
    }
  }

  void MappingRos::mapPubCallback(const ros::TimerEvent& e) {
    PointsPtr static_cloud(new Points);
    static_cloud->points.reserve(cloud_world->points.size());
    const double dynamic_radius_xy = 0.5;
    const double dynamic_radius_xy_sq = dynamic_radius_xy * dynamic_radius_xy;
    const double z_thresh = 0.5;

    for (const auto& pt : cloud_world->points)
    {
      bool is_dynamic = false;

      for (const auto& tracker : trackers)
      {
        // if (tracker->age < 3) continue;

        double dx = pt.x - tracker->pos()(0);
        double dy = pt.y - tracker->pos()(1);
        double dz = fabs(pt.z - tracker->pos()(2));

        if (dx*dx + dy*dy < dynamic_radius_xy_sq && dz < z_thresh)
        {
          is_dynamic = true;
          break;
        }
      }

      if (!is_dynamic)
        static_cloud->points.push_back(pt);
    }

    sensor_msgs::PointCloud2 static_msg;
    pcl::toROSMsg(*static_cloud, static_msg);
    static_msg.header.frame_id = "world";
    static_msg.header.stamp = ros::Time::now();
    staticMapPub.publish(static_msg);
  }

  void MappingRos::visualizeDbscan() {
    visualization_msgs::MarkerArray boxes;
    
    for (size_t i = 0; i < detections.size(); ++i) {
      float center_x = (detections[i].min_pt.x + detections[i].max_pt.x) / 2.0;
      float center_y = (detections[i].min_pt.y + detections[i].max_pt.y) / 2.0;
      float center_z = (detections[i].min_pt.z + detections[i].max_pt.z) / 2.0;

      float size_x = detections[i].max_pt.x - detections[i].min_pt.x;
      float size_y = detections[i].max_pt.y - detections[i].min_pt.y;
      float size_z = detections[i].max_pt.z - detections[i].min_pt.z;

      visualization_msgs::Marker cubeMarker;
      cubeMarker.id = i;
      cubeMarker.header.stamp = ros::Time::now();
      cubeMarker.header.frame_id = "world";
      cubeMarker.ns = "dbscan_cubes";

      cubeMarker.pose.position.x = center_x;
      cubeMarker.pose.position.y = center_y;
      cubeMarker.pose.position.z = center_z;
      
      cubeMarker.pose.orientation.x = 0.0;
      cubeMarker.pose.orientation.y = 0.0;
      cubeMarker.pose.orientation.z = 0.0;
      cubeMarker.pose.orientation.w = 1.0;
   
      cubeMarker.type = visualization_msgs::Marker::CUBE;
      cubeMarker.action = visualization_msgs::Marker::ADD;

      cubeMarker.scale.x = size_x;
      cubeMarker.scale.y = size_y;
      cubeMarker.scale.z = size_z;
      
      cubeMarker.color.r = 0.6;  
      cubeMarker.color.g = 0.8;  
      cubeMarker.color.b = 1.0;  
      cubeMarker.color.a = 0.3;

      cubeMarker.lifetime = ros::Duration(0.1);
      boxes.markers.push_back(cubeMarker);
    }
    clusterPub.publish(boxes);
  }

  void MappingRos::visualizeFunction(const std::vector<int> tra_id){
    visualization_msgs::MarkerArray idTexts;
    visualization_msgs::MarkerArray poses;
    visualization_msgs::MarkerArray boxes;
    visualization_msgs::MarkerArray sectorArray;
    obj_state_msgs::ObjectsStates states;
    
    for (const int& idx : tra_id) {
      for (size_t i = 0; i < trackers.size(); ++i)
      {
        if (idx == trackers[i]->id)
        {
          // std::cout <<  "trackers[i]->tra_pts->size(): " << trackers[i]->tra_pts->size() << std::endl;
          visualization_msgs::Marker edgeMarker;
          edgeMarker.id = trackers[i]->id;
          edgeMarker.header.stamp = ros::Time::now();
          edgeMarker.header.frame_id = "world";
          edgeMarker.pose.orientation.w = 1.00;
          edgeMarker.lifetime = ros::Duration(0.1);
          edgeMarker.type = visualization_msgs::Marker::LINE_STRIP;
          edgeMarker.action = visualization_msgs::Marker::ADD;
          edgeMarker.ns = "edge";
          edgeMarker.color.r = 0.00;
          edgeMarker.color.g = 1.00;
          edgeMarker.color.b = 0.00;
          edgeMarker.color.a = 0.80;
          edgeMarker.scale.x = 0.1;
          geometry_msgs::Point point[8];
          point[0].x = trackers[i]->min_pt.x; point[0].y = trackers[i]->max_pt.y; point[0].z = trackers[i]->max_pt.z;
          point[1].x = trackers[i]->min_pt.x; point[1].y = trackers[i]->min_pt.y; point[1].z = trackers[i]->max_pt.z;
          point[2].x = trackers[i]->min_pt.x; point[2].y = trackers[i]->min_pt.y; point[2].z = trackers[i]->min_pt.z;
          point[3].x = trackers[i]->min_pt.x; point[3].y = trackers[i]->max_pt.y; point[3].z = trackers[i]->min_pt.z;
          point[4].x = trackers[i]->max_pt.x; point[4].y = trackers[i]->max_pt.y; point[4].z = trackers[i]->min_pt.z;
          point[5].x = trackers[i]->max_pt.x; point[5].y = trackers[i]->min_pt.y; point[5].z = trackers[i]->min_pt.z;
          point[6].x = trackers[i]->max_pt.x; point[6].y = trackers[i]->min_pt.y; point[6].z = trackers[i]->max_pt.z;
          point[7].x = trackers[i]->max_pt.x; point[7].y = trackers[i]->max_pt.y; point[7].z = trackers[i]->max_pt.z;
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

          visualization_msgs::Marker poseMarker;
          ObjectState state;
          state.pos = trackers[i]->pos();
          state.pos[2] = 0.4; // FIX2D
          state.vel = trackers[i]->vel();
          state.vel[2] = 0; // FIX2D
          poseMarker.id = 1000 + trackers[i]->id;
          poseMarker.header.stamp = ros::Time::now();
          poseMarker.header.frame_id = "world";
          poseMarker.lifetime = ros::Duration(0.1);
          poseMarker.type = visualization_msgs::Marker::ARROW;
          poseMarker.action = visualization_msgs::Marker::ADD;
          poseMarker.ns = "objectpose";
          poseMarker.color.r = 0.00;
          poseMarker.color.g = 1.00;
          poseMarker.color.b = 0.00;
          poseMarker.color.a = 1.00;
          poseMarker.scale.x = 0.10;
          poseMarker.scale.y = 0.18;
          poseMarker.scale.z = 0.30;
          poseMarker.pose.orientation.w = 1.0;
          geometry_msgs::Point arrow[2];
          arrow[0].x = state.pos[0]; arrow[0].y = state.pos[1]; arrow[0].z = state.pos[2];
          arrow[1].x = state.pos[0] + 1.0*state.vel[0]/state.vel.norm(); 
          arrow[1].y = state.pos[1] + 1.0*state.vel[1]/state.vel.norm(); 
          arrow[1].z = state.pos[2] + 1.0*state.vel[2]/state.vel.norm();

          poseMarker.points.push_back(arrow[0]);
          poseMarker.points.push_back(arrow[1]);
          if (state.vel.norm() > 0.01)
            poses.markers.push_back(poseMarker);
          
          obj_state_msgs::State statemsg;
          statemsg.header.stamp = ros::Time::now();
          statemsg.position.x = state.pos[0]; statemsg.position.y = state.pos[1]; statemsg.position.z = state.pos[2];
          statemsg.velocity.x = state.vel[0]; statemsg.velocity.y = state.vel[1]; statemsg.velocity.z = state.vel[2];
          statemsg.size.x = 0; statemsg.size.y = 0; statemsg.size.z = 0;
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
          visualization_msgs::Marker sectorMarker;
          sectorMarker.id = 2000 + trackers[i]->id;
          sectorMarker.header.stamp = ros::Time::now();
          sectorMarker.header.frame_id = "world";
          sectorMarker.lifetime = ros::Duration(0.1);
          sectorMarker.type = visualization_msgs::Marker::LINE_STRIP;
          sectorMarker.action = visualization_msgs::Marker::ADD;
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

              geometry_msgs::Point q;
              q.x = p_arc.x();
              q.y = p_arc.y();
              q.z = p_arc.z();
              sectorMarker.points.push_back(q);
          }

    
          geometry_msgs::Point p0;
          p0.x = p.x(); p0.y = p.y(); p0.z = p.z();
          sectorMarker.points.insert(sectorMarker.points.begin(), p0);
          sectorMarker.points.push_back(p0);

          sectorArray.markers.push_back(sectorMarker);
          
          // ================= ID TEXT MARKER =================
          visualization_msgs::Marker textMarker;
          textMarker.header.frame_id = "world";
          textMarker.header.stamp = ros::Time::now();
          textMarker.ns = "object_id";
          textMarker.id = 3000 + trackers[i]->id; 
          textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          textMarker.action = visualization_msgs::Marker::ADD;
          textMarker.lifetime = ros::Duration(0.1);

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
    const obj_state_msgs::ObjectsStates& states)
    {
      if (!states.states.empty())
        statesPub.publish(states);
    }

    inline void MappingRos::publishBoxes(
        visualization_msgs::MarkerArray& boxes)
    {
      if (!boxes.markers.empty())
        edgePub.publish(boxes);
    }

    inline void MappingRos::publishObjectPoses(
        visualization_msgs::MarkerArray& poses)
    {
      if (!poses.markers.empty())
        objectPosePub.publish(poses);
    }

    inline void MappingRos::publishIDTexts(
        visualization_msgs::MarkerArray& idTexts)
    {
      if (!idTexts.markers.empty())
        idTextPub.publish(idTexts);
    }

    inline void MappingRos::publishSectors(
        visualization_msgs::MarkerArray& sectors)
    {
      if (!sectors.markers.empty())
        sectorPub.publish(sectors);
    }
  
}
