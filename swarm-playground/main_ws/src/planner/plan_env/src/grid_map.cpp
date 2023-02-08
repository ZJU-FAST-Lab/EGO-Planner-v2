#include "plan_env/grid_map.h"

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  // double x_size, y_size, z_size;
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);
  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);

  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range3d_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range3d_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range3d_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
  node_.param("grid_map/enable_virtual_wall", mp_.enable_virtual_walll_, false);
  node_.param("grid_map/virtual_ceil", mp_.virtual_ceil_, -1.0);
  node_.param("grid_map/virtual_ground", mp_.virtual_ground_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, 0.1);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/fading_time", mp_.fading_time_, 1000.0);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, 0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);

  mp_.inf_grid_ = ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);
  if (mp_.inf_grid_ > 4)
  {
    mp_.inf_grid_ = 4;
    mp_.resolution_ = mp_.obstacles_inflation_ / mp_.inf_grid_;
    ROS_WARN("Inflation is too big, which will cause siginificant computation! Resolution enalrged to %f automatically.", mp_.resolution_);
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
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
  extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this);
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.032), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.125), &GridMap::visCallback, this);
  if (mp_.fading_time_ > 0)
    fading_timer_ = node_.createTimer(ros::Duration(0.5), &GridMap::fadingCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.last_occ_update_time_.fromSec(0);

  md_.flag_have_ever_received_depth_ = false;
  md_.flag_depth_odom_timeout_ = false;
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!checkDepthOdomNeedupdate())
    return;

  /* update occupancy */
  ros::Time t1, t2, t3, t4, t5;
  t1 = ros::Time::now();

  moveRingBuffer();
  t2 = ros::Time::now();

  projectDepthImage();
  t3 = ros::Time::now();

  if (md_.proj_points_cnt_ > 0)
  {
    raycastProcess();
    t4 = ros::Time::now();

    clearAndInflateLocalMap();
    t5 = ros::Time::now();

    if (mp_.show_occ_time_)
    {
      cout << setprecision(7);
      cout << "t2=" << (t2 - t1).toSec() << " t3=" << (t3 - t2).toSec() << " t4=" << (t4 - t3).toSec() << " t5=" << (t5 - t4).toSec() << endl;

      static int updatetimes = 0;
      static double raycasttime = 0;
      static double max_raycasttime = 0;
      static double inflationtime = 0;
      static double max_inflationtime = 0;
      raycasttime += (t4 - t3).toSec();
      max_raycasttime = max(max_raycasttime, (t4 - t3).toSec());
      inflationtime += (t5 - t4).toSec();
      max_inflationtime = max(max_inflationtime, (t5 - t4).toSec());
      ++updatetimes;

      printf("Raycast(ms): cur t = %lf, avg t = %lf, max t = %lf\n", (t4 - t3).toSec() * 1000, raycasttime / updatetimes * 1000, max_raycasttime * 1000);
      printf("Infaltion(ms): cur t = %lf, avg t = %lf, max t = %lf\n", (t5 - t4).toSec() * 1000, inflationtime / updatetimes * 1000, max_inflationtime * 1000);
    }
  }

  md_.occ_need_update_ = false;
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{
  if (!mp_.have_initialized_)
    return;

  ros::Time t0 = ros::Time::now();
  publishMapInflate();
  publishMap();
  ros::Time t1 = ros::Time::now();

  if (mp_.show_occ_time_)
  {
    printf("Visualization(ms):%f\n", (t1 - t0).toSec() * 1000);
  }
}

void GridMap::fadingCallback(const ros::TimerEvent & /*event*/)
{
  const double reduce = (mp_.clamp_max_log_ - mp_.min_occupancy_log_) / (mp_.fading_time_ * 2); // function called at 2Hz
  const double low_thres = mp_.clamp_min_log_ + reduce;

  ros::Time t0 = ros::Time::now();
  for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
  {
    if (md_.occupancy_buffer_[i] > low_thres)
    {
      bool obs_flag = md_.occupancy_buffer_[i] >= mp_.min_occupancy_log_;
      md_.occupancy_buffer_[i] -= reduce;
      if (obs_flag && md_.occupancy_buffer_[i] < mp_.min_occupancy_log_)
      {
        Eigen::Vector3i idx = BufIdx2GlobalIdx(i);
        int inf_buf_idx = globalIdx2InfBufIdx(idx);
        changeInfBuf(false, inf_buf_idx, idx);
      }
    }
  }
  ros::Time t1 = ros::Time::now();

  if (mp_.show_occ_time_)
  {
    printf("Fading(ms):%f\n", (t1 - t0).toSec() * 1000);
  }
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  static bool first_flag = true;
  if (first_flag)
  {
    first_flag = false;
    md_.proj_points_.resize(md_.depth_image_.cols * md_.depth_image_.rows / mp_.skip_pixel_ / mp_.skip_pixel_);
  }

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();

  md_.occ_need_update_ = true;
  md_.flag_have_ever_received_depth_ = true;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{

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

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  static bool first_flag = true;
  if (first_flag)
  {
    first_flag = false;
    md_.proj_points_.resize(md_.depth_image_.cols * md_.depth_image_.rows / mp_.skip_pixel_ / mp_.skip_pixel_);
  }

  md_.occ_need_update_ = true;
  md_.flag_have_ever_received_depth_ = true;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.flag_have_ever_received_depth_)
  {
    indep_odom_sub_.shutdown();
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

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
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
      md_.occupancy_buffer_[buf_id] = mp_.clamp_max_log_;

      if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)
      {
        changeInfBuf(true, inf_buf_id, idx);
      }
    }
  }
}

void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

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

void GridMap::projectDepthImage()
{
  md_.proj_points_cnt_ = 0;

  uint16_t *row_ptr;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr) / mp_.k_depth_scaling_factor_;
        row_ptr = row_ptr + mp_.skip_pixel_;

        if (depth < 0.1)
          continue;

        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        md_.proj_points_[md_.proj_points_cnt_++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;

          md_.proj_points_[md_.proj_points_cnt_++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt_++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt_++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  md_.cache_voxel_cnt_ = 0;

  ros::Time t1, t2, t3;

  md_.raycast_num_ += 1;

  RayCaster raycaster;
  Eigen::Vector3d ray_pt, pt_w;

  int pts_num = 0;
  t1 = ros::Time::now();
  for (int i = 0; i < md_.proj_points_cnt_; ++i)
  {
    int vox_idx;
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInBuf(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);
      pts_num++;
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      pts_num++;
      vox_idx = setCacheOccupancy(pt_w, 1);
    }

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + Eigen::Vector3d(0.5, 0.5, 0.5)) * mp_.resolution_;

      pts_num++;
      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  t2 = ros::Time::now();

  for (int i = 0; i < md_.cache_voxel_cnt_; ++i)
  {

    int idx_ctns = globalIdx2BufIdx(md_.cache_voxel_[i]);

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      continue;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }

  t3 = ros::Time::now();

  if (mp_.show_occ_time_)
  {
    ROS_WARN("Raycast time: t2-t1=%f, t3-t2=%f, pts_num=%d", (t2 - t1).toSec(), (t3 - t2).toSec(), pts_num);
  }
}

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
          ROS_ERROR("Here should be 0!!! md_.occupancy_buffer_inflate_[id_buf_inf_clr]=%d", md_.occupancy_buffer_inflate_[id_buf_inf_clr]);
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
  if (md_.last_occ_update_time_.toSec() < 1.0)
  {
    md_.last_occ_update_time_ = ros::Time::now();
  }
  if (!md_.occ_need_update_)
  {
    if (md_.flag_have_ever_received_depth_ && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_)
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return false;
  }
  md_.last_occ_update_time_ = ros::Time::now();

  return true;
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  Eigen::Vector3d heading = (md_.camera_r_m_ * md_.cam2body_.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double lbz = mp_.enable_virtual_walll_ ? max(md_.ringbuffer_lowbound3d_(2), mp_.virtual_ground_) : md_.ringbuffer_lowbound3d_(2);
  double ubz = mp_.enable_virtual_walll_ ? min(md_.ringbuffer_upbound3d_(2), mp_.virtual_ceil_) : md_.ringbuffer_upbound3d_(2);
  if (md_.ringbuffer_upbound3d_(0) - md_.ringbuffer_lowbound3d_(0) > mp_.resolution_ && (md_.ringbuffer_upbound3d_(1) - md_.ringbuffer_lowbound3d_(1)) > mp_.resolution_ && (ubz - lbz) > mp_.resolution_)
    for (double xd = md_.ringbuffer_lowbound3d_(0) + mp_.resolution_ / 2; xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1) + mp_.resolution_ / 2; yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = lbz + mp_.resolution_ / 2; zd <= ubz; zd += mp_.resolution_)
        {
          Eigen::Vector3d relative_dir = (Eigen::Vector3d(xd, yd, zd) - md_.camera_pos_);
          if (heading.dot(relative_dir.normalized()) > 0.5)
          {
            if (md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))] >= mp_.min_occupancy_log_)
              cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          }
        }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate()
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  Eigen::Vector3d heading = (md_.camera_r_m_ * md_.cam2body_.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double lbz = mp_.enable_virtual_walll_ ? max(md_.ringbuffer_inf_lowbound3d_(2), mp_.virtual_ground_) : md_.ringbuffer_inf_lowbound3d_(2);
  double ubz = mp_.enable_virtual_walll_ ? min(md_.ringbuffer_inf_upbound3d_(2), mp_.virtual_ceil_) : md_.ringbuffer_inf_upbound3d_(2);
  if (md_.ringbuffer_inf_upbound3d_(0) - md_.ringbuffer_inf_lowbound3d_(0) > mp_.resolution_ &&
      (md_.ringbuffer_inf_upbound3d_(1) - md_.ringbuffer_inf_lowbound3d_(1)) > mp_.resolution_ && (ubz - lbz) > mp_.resolution_)
    for (double xd = md_.ringbuffer_inf_lowbound3d_(0) + mp_.resolution_ / 2; xd < md_.ringbuffer_inf_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_inf_lowbound3d_(1) + mp_.resolution_ / 2; yd < md_.ringbuffer_inf_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = lbz + mp_.resolution_ / 2; zd < ubz; zd += mp_.resolution_)
        {
          Eigen::Vector3d relative_dir = (Eigen::Vector3d(xd, yd, zd) - md_.camera_pos_);
          if (heading.dot(relative_dir.normalized()) > 0.5)
          {
            if (md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))])
              cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          }
        }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);
}

void GridMap::testIndexingCost()
{
  if (!mp_.have_initialized_)
    return;

  ros::Time t0 = ros::Time::now();
  double a = 0;
  int b = 0;
  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          b += x + y + z;
        }
  ros::Time t1 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          int id_buf_inf_clr = globalIdx2InfBufIdx(Eigen::Vector3i(x, y, z)); // 8396us = 7970
          b += id_buf_inf_clr;
          b += x + y + z;
        }
  ros::Time t2 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          Eigen::Vector3d pos = globalIdx2Pos(Eigen::Vector3i(x, y, z)); // 6553us = 6127
          a += pos.sum();
          b += x + y + z;
        }
  ros::Time t3 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound3d_(0); xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1); yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = md_.ringbuffer_lowbound3d_(2); zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.resolution_)
        {
          a += xd + yd + zd;
        }
  ros::Time t4 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound3d_(0); xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1); yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = md_.ringbuffer_lowbound3d_(2); zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.resolution_)
        {
          Eigen::Vector3i idx = pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)); // 7088us = 478us
          a += xd + yd + zd;
          b += idx.sum();
        }
  ros::Time t5 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound3d_(0); xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1); yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = md_.ringbuffer_lowbound3d_(2); zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.resolution_)
        {
          int id_buf_inf_clr = globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)));
          a += xd + yd + zd;
          b += id_buf_inf_clr;
        }
  ros::Time t6 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
    {
      b += i;
    }
  ros::Time t7 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
    {
      Eigen::Vector3i idx = BufIdx2GlobalIdx(i); // 36939
      b += i;
      b += idx.sum();
    }
  ros::Time t8 = ros::Time::now();

  int n = md_.occupancy_buffer_.size() * 10;

  cout << "a=" << a << " b=" << b << endl;
  printf("iter=%d, t1-t0=%f, t2-t1=%f, t3-t2=%f, t4-t3=%f, t5-t4=%f, t6-t5=%f, t7-t6=%f, t8-t7=%f\n", n, (t1 - t0).toSec(), (t2 - t1).toSec(), (t3 - t2).toSec(), (t4 - t3).toSec(), (t5 - t4).toSec(), (t6 - t5).toSec(), (t7 - t6).toSec(), (t8 - t7).toSec());
  printf("globalIdx2InfBufIdx():%fns(1.88), globalIdx2Pos():%fns(0.70), pos2GlobalIdx():%fns(1.11), globalIdx2InfBufIdx(pos2GlobalIdx()):%fns(3.56), BufIdx2GlobalIdx():%fns(10.05)\n",
         ((t2 - t1) - (t1 - t0)).toSec() * 1e9 / n, ((t3 - t2) - (t1 - t0)).toSec() * 1e9 / n, ((t5 - t4) - (t4 - t3)).toSec() * 1e9 / n, ((t6 - t5) - (t4 - t3)).toSec() * 1e9 / n, ((t8 - t7) - (t7 - t6)).toSec() * 1e9 / n);
}