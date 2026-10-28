#include "plan_env/grid_map.h"

void GridMap::initMap(rclcpp::Node::SharedPtr nh)
{
  node_ = std::move(nh);
  auto param = [this](const std::string &name, auto default_value) {
    return node_->declare_parameter<decltype(default_value)>(name, default_value);
  };

  /* get parameter */
  // double x_size, y_size, z_size;
  mp_.pose_type_ = param("grid_map.pose_type", 1);
  mp_.frame_id_ = param("grid_map.frame_id", std::string("world"));
  mp_.odom_depth_timeout_ = param("grid_map.odom_depth_timeout", 1.0);
  mp_.resolution_ = param("grid_map.resolution", 0.1);
  // Local update range
  mp_.local_update_range3d_(0) = param("grid_map.local_update_range_x", 5.5);
  mp_.local_update_range3d_(1) = param("grid_map.local_update_range_y", 5.5);
  mp_.local_update_range3d_(2) = param("grid_map.local_update_range_z", 2.0);
  // Obstacle inflation radius
  mp_.obstacles_inflation_ = param("grid_map.obstacles_inflation", 0.1);
  mp_.enable_virtual_walll_ = param("grid_map.enable_virtual_wall", true);
  mp_.virtual_ceil_ = param("grid_map.virtual_ceil", 5.0);
  mp_.virtual_ground_ = param("grid_map.virtual_ground", -0.1);

  mp_.fx_ = param("grid_map.fx", 387.229248046875);
  mp_.fy_ = param("grid_map.fy", 387.229248046875);
  mp_.cx_ = param("grid_map.cx", 321.04638671875);
  mp_.cy_ = param("grid_map.cy", 243.44969177246094);

  mp_.use_depth_filter_ = param("grid_map.use_depth_filter", true);
  mp_.depth_filter_tolerance_ = param("grid_map.depth_filter_tolerance", 0.15);
  mp_.depth_filter_mindist_ = param("grid_map.depth_filter_mindist", 0.1);
  mp_.depth_filter_margin_ = param("grid_map.depth_filter_margin", 2);
  mp_.k_depth_scaling_factor_ = param("grid_map.k_depth_scaling_factor", 1000.0);
  mp_.skip_pixel_ = param("grid_map.skip_pixel", 2);

  mp_.static_hit_thres_ = param("grid_map.static_hit_thres", 8);
  mp_.p_hit_ = param("grid_map.p_hit", 0.70);
  mp_.p_miss_ = param("grid_map.p_miss", 0.35);
  mp_.p_min_ = param("grid_map.p_min", 0.12);
  mp_.p_max_ = param("grid_map.p_max", 0.97);
  mp_.p_occ_ = param("grid_map.p_occ", 0.80);
  mp_.fading_time_ = param("grid_map.fading_time", 1000.0);
  mp_.min_ray_length_ = param("grid_map.min_ray_length", 0.1);
  mp_.show_occ_time_ = param("grid_map.show_occ_time", false);

  mp_.inf_grid_ = ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);
  if (mp_.inf_grid_ > 4)
  {
    mp_.inf_grid_ = 4;
    mp_.resolution_ = mp_.obstacles_inflation_ / mp_.inf_grid_;
    RCLCPP_WARN(node_->get_logger(), "Inflation is too big, which will cause siginificant computation! Resolution enalrged to %f automatically.", mp_.resolution_);
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.local_update_range3i_ = (mp_.local_update_range3d_ * mp_.resolution_inv_).array().ceil().cast<int>();
  mp_.local_update_range3d_ = mp_.local_update_range3i_.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_size3i_ = 2 * mp_.local_update_range3i_;
  md_.ringbuffer_inf_size3i_ = md_.ringbuffer_size3i_ + Eigen::Vector3i(2 * mp_.inf_grid_, 2 * mp_.inf_grid_, 2 * mp_.inf_grid_);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  // initialize data buffers
  Eigen::Vector3i map_voxel_num3i = 2 * mp_.local_update_range3i_;
  int buffer_size = map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
  int buffer_inf_size = (map_voxel_num3i(0) + 2 * mp_.inf_grid_) * (map_voxel_num3i(1) + 2 * mp_.inf_grid_) * (map_voxel_num3i(2) + 2 * mp_.inf_grid_);
  md_.ringbuffer_origin3i_ = Eigen::Vector3i(0, 0, 0);
  md_.ringbuffer_inf_origin3i_ = Eigen::Vector3i(0, 0, 0);

  // Log-odds occupancy buffer
  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_);
  md_.occupancy_buffer_inflate_ = vector<uint16_t>(buffer_inf_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);
  md_.cache_voxel_ = vector<Eigen::Vector3i>(buffer_size, Eigen::Vector3i(0, 0, 0));

  md_.raycast_num_ = 0;
  md_.proj_points_cnt_ = 0;
  md_.cache_voxel_cnt_ = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */
  // depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/camera/depth/image_rect_raw", 50));
  // extrinsic_sub_ = node_.subscribe<nav_msgs::msg::Odometry>(
  //     "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  // if (mp_.pose_type_ == POSE_STAMPED)
  // {
  //   pose_sub_.reset(
  //       new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

  //   sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
  //       SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
  //   sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  // }
  // else if (mp_.pose_type_ == ODOMETRY)
  // {

  //   sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
  //       SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
  //   sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  // }

  // use odometry and point cloud
  indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", rclcpp::SensorDataQoS(),
      std::bind(&GridMap::odomCallback, this, std::placeholders::_1));
  indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/static_map", rclcpp::SensorDataQoS(),
      std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));

  // Visualization timer
  vis_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&GridMap::visCallback, this));
  // Fading mechanism
  if (mp_.fading_time_ > 0)
    fading_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&GridMap::fadingCallback, this));

  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);

  // md_.occ_need_update_ = false;
  md_.occ_need_update_ = true; // FIXME
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.last_occ_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  md_.flag_have_ever_received_depth_ = false;
  md_.flag_depth_odom_timeout_ = false;
}

// Publish occupancy map (raw and inflated)
void GridMap::visCallback()
{
  if (!mp_.have_initialized_)
    return;

  rclcpp::Time t0 = node_->now();
  publishMapInflate();
  publishMap();
  rclcpp::Time t1 = node_->now();

  if (mp_.show_occ_time_)
  {
    printf("Visualization(ms):%f\n", (t1 - t0).seconds() * 1000);
  }
}

// Fading: reduce occupancy probability over time to prevent noise accumulation
void GridMap::fadingCallback()
{
  const double reduce = (mp_.clamp_max_log_ - mp_.min_occupancy_log_) / (mp_.fading_time_ * 2); // function called at 2Hz
  const double low_thres = mp_.clamp_min_log_ + reduce;

  rclcpp::Time t0 = node_->now();
  for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
  {
    const bool was_occupied = md_.occupancy_buffer_[i] >= mp_.min_occupancy_log_;
    if (md_.occupancy_buffer_[i] > low_thres)
    {
      md_.occupancy_buffer_[i] -= reduce;
    }
    // FIXME
    if (md_.count_hit_[i] == 0)
    {
      // Not observed recently, decay faster
      md_.occupancy_buffer_[i] = std::max(
          mp_.clamp_min_log_, md_.occupancy_buffer_[i] - 2.0 * reduce);
    }
    else
    {
      md_.count_hit_[i] = 0; // Reset for next cycle
    }
    // Both decay paths must remove inflation when occupancy expires.
    if (was_occupied && md_.occupancy_buffer_[i] < mp_.min_occupancy_log_)
    {
      const Eigen::Vector3i idx = BufIdx2GlobalIdx(i);
      changeInfBuf(false, globalIdx2InfBufIdx(idx), idx);
    }
  }
  rclcpp::Time t1 = node_->now();

  if (mp_.show_occ_time_)
  {
    printf("Fading(ms):%f\n", (t1 - t0).seconds() * 1000);
  }
}

void GridMap::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  if (md_.flag_have_ever_received_depth_)
  {
    indep_odom_sub_.reset();
    return;
  }

  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  md_.has_odom_ = true;
}

void GridMap::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr img)
{
  /* Note: no obstalce elimination in this function! */

  if (!md_.has_odom_)
  {
    std::cout << "grid_map: no odom!" << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  moveRingBuffer();
  // Keep inflation for existing occupied voxels across observations and hit
  // counter resets. New occupied voxels add inflation below; fading and the
  // moving ring buffer remove it when the corresponding occupancy is removed.

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;
  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
    if (p3d.array().isNaN().sum())
      continue;

    if (isInBuf(p3d))
    {
      /* inflate the point */
      Eigen::Vector3i idx = pos2GlobalIdx(p3d);
      int buf_id = globalIdx2BufIdx(idx);
      int inf_buf_id = globalIdx2InfBufIdx(idx);
      // md_.occupancy_buffer_[buf_id] = mp_.clamp_max_log_;

      // if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)
      // {
      //   changeInfBuf(true, inf_buf_id, idx);
      // }

      // Hit count FIXME
      md_.count_hit_[buf_id]++;
      md_.count_hit_and_miss_[buf_id]++;

      // Only mark stable voxels as occupied
      if (md_.count_hit_[buf_id] >= mp_.static_hit_thres_)
      {
        md_.occupancy_buffer_[buf_id] =
            std::min(md_.occupancy_buffer_[buf_id] + mp_.prob_hit_log_,
                    mp_.clamp_max_log_);

        if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG &&
            md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)
        {
          changeInfBuf(true, inf_buf_id, idx);
        }
      }
    }
  }
}

// Dynamic local map: keep map centered around camera position
void GridMap::moveRingBuffer()
{
  if (!mp_.have_initialized_)
    initMapBoundary();

  Eigen::Vector3i center_new = pos2GlobalIdx(md_.camera_pos_);
  Eigen::Vector3i ringbuffer_lowbound3i_new = center_new - mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_lowbound3d_new = ringbuffer_lowbound3i_new.cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_upbound3i_new = center_new + mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_upbound3d_new = ringbuffer_upbound3i_new.cast<double>() * mp_.resolution_;
  ringbuffer_upbound3i_new -= Eigen::Vector3i(1, 1, 1);

  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_new = ringbuffer_lowbound3i_new - inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_new = ringbuffer_lowbound3d_new - inf_grid3d;
  Eigen::Vector3i ringbuffer_inf_upbound3i_new = ringbuffer_upbound3i_new + inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_upbound3d_new = ringbuffer_upbound3d_new + inf_grid3d;

  if (center_new(0) < md_.center_last3i_(0))
    clearBuffer(0, ringbuffer_upbound3i_new(0));
  if (center_new(0) > md_.center_last3i_(0))
    clearBuffer(1, ringbuffer_lowbound3i_new(0));
  if (center_new(1) < md_.center_last3i_(1))
    clearBuffer(2, ringbuffer_upbound3i_new(1));
  if (center_new(1) > md_.center_last3i_(1))
    clearBuffer(3, ringbuffer_lowbound3i_new(1));
  if (center_new(2) < md_.center_last3i_(2))
    clearBuffer(4, ringbuffer_upbound3i_new(2));
  if (center_new(2) > md_.center_last3i_(2))
    clearBuffer(5, ringbuffer_lowbound3i_new(2));

  for (int i = 0; i < 3; ++i)
  {
    while (md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i))
    {
      md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
    }
    while (md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i))
    {
      md_.ringbuffer_origin3i_(i) -= md_.ringbuffer_size3i_(i);
    }

    while (md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i))
    {
      md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
    }
    while (md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3i_(i))
    {
      md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
    }
  }

  md_.center_last3i_ = center_new;
  md_.ringbuffer_lowbound3i_ = ringbuffer_lowbound3i_new;
  md_.ringbuffer_lowbound3d_ = ringbuffer_lowbound3d_new;
  md_.ringbuffer_upbound3i_ = ringbuffer_upbound3i_new;
  md_.ringbuffer_upbound3d_ = ringbuffer_upbound3d_new;
  md_.ringbuffer_inf_lowbound3i_ = ringbuffer_inf_lowbound3i_new;
  md_.ringbuffer_inf_lowbound3d_ = ringbuffer_inf_lowbound3d_new;
  md_.ringbuffer_inf_upbound3i_ = ringbuffer_inf_upbound3i_new;
  md_.ringbuffer_inf_upbound3d_ = ringbuffer_inf_upbound3d_new;
}

// Update inflated map
void GridMap::clearAndInflateLocalMap()
{
  for (int i = 0; i < md_.cache_voxel_cnt_; ++i)
  {
    Eigen::Vector3i idx = md_.cache_voxel_[i];
    int buf_id = globalIdx2BufIdx(idx);
    int inf_buf_id = globalIdx2InfBufIdx(idx);

    if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)
    {
      changeInfBuf(true, inf_buf_id, idx);
    }

    if (md_.occupancy_buffer_inflate_[inf_buf_id] >= GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] < mp_.min_occupancy_log_)
    {
      changeInfBuf(false, inf_buf_id, idx);
    }
  }
}

void GridMap::initMapBoundary()
{
  mp_.have_initialized_ = true;

  md_.center_last3i_ = pos2GlobalIdx(md_.camera_pos_);

  md_.ringbuffer_lowbound3i_ = md_.center_last3i_ - mp_.local_update_range3i_;
  md_.ringbuffer_lowbound3d_ = md_.ringbuffer_lowbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ = md_.center_last3i_ + mp_.local_update_range3i_;
  md_.ringbuffer_upbound3d_ = md_.ringbuffer_upbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ -= Eigen::Vector3i(1, 1, 1);

  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_inf_lowbound3i_ = md_.ringbuffer_lowbound3i_ - inf_grid3i;
  md_.ringbuffer_inf_lowbound3d_ = md_.ringbuffer_lowbound3d_ - inf_grid3d;
  md_.ringbuffer_inf_upbound3i_ = md_.ringbuffer_upbound3i_ + inf_grid3i;
  md_.ringbuffer_inf_upbound3d_ = md_.ringbuffer_upbound3d_ + inf_grid3d;

  // cout << "md_.ringbuffer_lowbound3i_=" << md_.ringbuffer_lowbound3i_.transpose() << " md_.ringbuffer_lowbound3d_=" << md_.ringbuffer_lowbound3d_.transpose() << " md_.ringbuffer_upbound3i_=" << md_.ringbuffer_upbound3i_.transpose() << " md_.ringbuffer_upbound3d_=" << md_.ringbuffer_upbound3d_.transpose() << endl;

  for (int i = 0; i < 3; ++i)
  {
    while (md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i))
    {
      md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
    }
    while (md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i))
    {
      md_.ringbuffer_origin3i_(i) -= md_.ringbuffer_size3i_(i);
    }

    while (md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i))
    {
      md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
    }
    while (md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3i_(i))
    {
      md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
    }
  }

#if GRID_MAP_NEW_PLATFORM_TEST
  testIndexingCost();
#endif
}

void GridMap::clearBuffer(char casein, int bound)
{
  for (int x = (casein == 0 ? bound : md_.ringbuffer_lowbound3i_(0)); x <= (casein == 1 ? bound : md_.ringbuffer_upbound3i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.ringbuffer_lowbound3i_(1)); y <= (casein == 3 ? bound : md_.ringbuffer_upbound3i_(1)); ++y)
      for (int z = (casein == 4 ? bound : md_.ringbuffer_lowbound3i_(2)); z <= (casein == 5 ? bound : md_.ringbuffer_upbound3i_(2)); ++z)
      {
        Eigen::Vector3i id_global(x, y, z);
        int id_buf = globalIdx2BufIdx(id_global);
        int id_buf_inf = globalIdx2InfBufIdx(id_global);
        Eigen::Vector3i id_global_inf_clr((casein == 0 ? x + mp_.inf_grid_ : (casein == 1 ? x - mp_.inf_grid_ : x)),
                                          (casein == 2 ? y + mp_.inf_grid_ : (casein == 3 ? y - mp_.inf_grid_ : y)),
                                          (casein == 4 ? z + mp_.inf_grid_ : (casein == 5 ? z - mp_.inf_grid_ : z)));
        // int id_buf_inf_clr = globalIdx2InfBufIdx(id_global_inf_clr);

        // md_.occupancy_buffer_inflate_[id_buf_inf_clr] = 0;
        md_.count_hit_[id_buf] = 0;
        md_.count_hit_and_miss_[id_buf] = 0;
        md_.flag_traverse_[id_buf] = md_.raycast_num_;
        md_.flag_rayend_[id_buf] = md_.raycast_num_;
        md_.occupancy_buffer_[id_buf] = mp_.clamp_min_log_;

        if (md_.occupancy_buffer_inflate_[id_buf_inf] > GRID_MAP_OBS_FLAG)
        {
          changeInfBuf(false, id_buf_inf, id_global);
        }
      }

#if GRID_MAP_NEW_PLATFORM_TEST
  for (int x = (casein == 0 ? bound : md_.ringbuffer_lowbound3i_(0)); x <= (casein == 1 ? bound : md_.ringbuffer_upbound3i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.ringbuffer_lowbound3i_(1)); y <= (casein == 3 ? bound : md_.ringbuffer_upbound3i_(1)); ++y)
      for (int z = (casein == 4 ? bound : md_.ringbuffer_lowbound3i_(2)); z <= (casein == 5 ? bound : md_.ringbuffer_upbound3i_(2)); ++z)
      {
        Eigen::Vector3i id_global_inf_clr((casein == 0 ? x + mp_.inf_grid_ : (casein == 1 ? x - mp_.inf_grid_ : x)),
                                          (casein == 2 ? y + mp_.inf_grid_ : (casein == 3 ? y - mp_.inf_grid_ : y)),
                                          (casein == 4 ? z + mp_.inf_grid_ : (casein == 5 ? z - mp_.inf_grid_ : z)));
        int id_buf_inf_clr = globalIdx2InfBufIdx(id_global_inf_clr);
        if (md_.occupancy_buffer_inflate_[id_buf_inf_clr] != 0)
        {
          RCLCPP_ERROR(node_->get_logger(), "Here should be 0!!! md_.occupancy_buffer_inflate_[id_buf_inf_clr]=%d", md_.occupancy_buffer_inflate_[id_buf_inf_clr]);
        }
      }
#endif
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = md_.ringbuffer_upbound3d_ - camera_pt;
  Eigen::Vector3d min_tc = md_.ringbuffer_lowbound3d_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

bool GridMap::checkDepthOdomNeedupdate()
{
  if (md_.last_occ_update_time_.seconds() < 1.0)
  {
    md_.last_occ_update_time_ = node_->now();
  }
  if (!md_.occ_need_update_)
  {
    if (md_.flag_have_ever_received_depth_ && (node_->now() - md_.last_occ_update_time_).seconds() > mp_.odom_depth_timeout_)
    {
      RCLCPP_ERROR(node_->get_logger(), "odom or depth lost! node_->now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                node_->now().seconds(), md_.last_occ_update_time_.seconds(), mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return false;
  }
  md_.last_occ_update_time_ = node_->now();

  return true;
}

void GridMap::publishMap()
{

  if (map_pub_->get_subscription_count() <= 0)
    return;

  // Eigen::Vector3d heading = (md_.camera_r_m_ * md_.cam2body_.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0); // FIXME
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double lbz = mp_.enable_virtual_walll_ ? max(md_.ringbuffer_lowbound3d_(2), mp_.virtual_ground_) : md_.ringbuffer_lowbound3d_(2);
  double ubz = mp_.enable_virtual_walll_ ? min(md_.ringbuffer_upbound3d_(2), mp_.virtual_ceil_) : md_.ringbuffer_upbound3d_(2);
  if (md_.ringbuffer_upbound3d_(0) - md_.ringbuffer_lowbound3d_(0) > mp_.resolution_ && (md_.ringbuffer_upbound3d_(1) - md_.ringbuffer_lowbound3d_(1)) > mp_.resolution_ && (ubz - lbz) > mp_.resolution_)
    for (double xd = md_.ringbuffer_lowbound3d_(0) + mp_.resolution_ / 2; xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1) + mp_.resolution_ / 2; yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = lbz + mp_.resolution_ / 2; zd <= ubz; zd += mp_.resolution_)
        {
          // if (heading.dot(relative_dir.normalized()) > 0.5) // FIXME
          // {
            if (md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))] >= mp_.min_occupancy_log_)
              cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          // }
        }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate()
{

  if (map_inf_pub_->get_subscription_count() <= 0)
    return;

  // Eigen::Vector3d heading = (md_.camera_r_m_ * md_.cam2body_.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double lbz = mp_.enable_virtual_walll_ ? max(md_.ringbuffer_inf_lowbound3d_(2), mp_.virtual_ground_) : md_.ringbuffer_inf_lowbound3d_(2);
  double ubz = mp_.enable_virtual_walll_ ? min(md_.ringbuffer_inf_upbound3d_(2), mp_.virtual_ceil_) : md_.ringbuffer_inf_upbound3d_(2);
  if (md_.ringbuffer_inf_upbound3d_(0) - md_.ringbuffer_inf_lowbound3d_(0) > mp_.resolution_ &&
      (md_.ringbuffer_inf_upbound3d_(1) - md_.ringbuffer_inf_lowbound3d_(1)) > mp_.resolution_ && (ubz - lbz) > mp_.resolution_)
    for (double xd = md_.ringbuffer_inf_lowbound3d_(0) + mp_.resolution_ / 2; xd < md_.ringbuffer_inf_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_inf_lowbound3d_(1) + mp_.resolution_ / 2; yd < md_.ringbuffer_inf_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = lbz + mp_.resolution_ / 2; zd < ubz; zd += mp_.resolution_)
        {
          
            if (md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))])
              cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          
        }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_->publish(cloud_msg);
}

void GridMap::testIndexingCost()
{
  if (!mp_.have_initialized_)
    return;

  rclcpp::Time t0 = node_->now();
  double a = 0;
  int b = 0;
  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          b += x + y + z;
        }
  rclcpp::Time t1 = node_->now();

  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          int id_buf_inf_clr = globalIdx2InfBufIdx(Eigen::Vector3i(x, y, z)); // 8396us = 7970
          b += id_buf_inf_clr;
          b += x + y + z;
        }
  rclcpp::Time t2 = node_->now();

  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          Eigen::Vector3d pos = globalIdx2Pos(Eigen::Vector3i(x, y, z)); // 6553us = 6127
          a += pos.sum();
          b += x + y + z;
        }
  rclcpp::Time t3 = node_->now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound3d_(0); xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1); yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = md_.ringbuffer_lowbound3d_(2); zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.resolution_)
        {
          a += xd + yd + zd;
        }
  rclcpp::Time t4 = node_->now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound3d_(0); xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1); yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = md_.ringbuffer_lowbound3d_(2); zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.resolution_)
        {
          Eigen::Vector3i idx = pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)); // 7088us = 478us
          a += xd + yd + zd;
          b += idx.sum();
        }
  rclcpp::Time t5 = node_->now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound3d_(0); xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1); yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = md_.ringbuffer_lowbound3d_(2); zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.resolution_)
        {
          int id_buf_inf_clr = globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)));
          a += xd + yd + zd;
          b += id_buf_inf_clr;
        }
  rclcpp::Time t6 = node_->now();

  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
    {
      b += i;
    }
  rclcpp::Time t7 = node_->now();

  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
    {
      Eigen::Vector3i idx = BufIdx2GlobalIdx(i); // 36939
      b += i;
      b += idx.sum();
    }
  rclcpp::Time t8 = node_->now();

  int n = md_.occupancy_buffer_.size() * 10;

  cout << "a=" << a << " b=" << b << endl;
  printf("iter=%d, t1-t0=%f, t2-t1=%f, t3-t2=%f, t4-t3=%f, t5-t4=%f, t6-t5=%f, t7-t6=%f, t8-t7=%f\n", n, (t1 - t0).seconds(), (t2 - t1).seconds(), (t3 - t2).seconds(), (t4 - t3).seconds(), (t5 - t4).seconds(), (t6 - t5).seconds(), (t7 - t6).seconds(), (t8 - t7).seconds());
  printf("globalIdx2InfBufIdx():%fns(1.88), globalIdx2Pos():%fns(0.70), pos2GlobalIdx():%fns(1.11), globalIdx2InfBufIdx(pos2GlobalIdx()):%fns(3.56), BufIdx2GlobalIdx():%fns(10.05)\n",
         ((t2 - t1) - (t1 - t0)).seconds() * 1e9 / n, ((t3 - t2) - (t1 - t0)).seconds() * 1e9 / n, ((t5 - t4) - (t4 - t3)).seconds() * 1e9 / n, ((t6 - t5) - (t4 - t3)).seconds() * 1e9 / n, ((t8 - t7) - (t7 - t6)).seconds() * 1e9 / n);
}
