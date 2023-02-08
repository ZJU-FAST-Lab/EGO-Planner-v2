#ifndef _GRID_MAP_
#define _GRID_MAP_

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))
#define GRID_MAP_OBS_FLAG 32767
#define GRID_MAP_NEW_PLATFORM_TEST false

using namespace std;

// constant parameters

struct MappingParameters
{
  bool have_initialized_ = false;

  /* map properties */
  Eigen::Vector3d local_update_range3d_;
  Eigen::Vector3i local_update_range3i_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  int inf_grid_;
  string frame_id_;
  int pose_type_;
  bool enable_virtual_walll_;
  double virtual_ceil_, virtual_ground_;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* time out */
  double odom_depth_timeout_;

  /* depth image projection filtering */
  bool use_depth_filter_;
  double depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;                                           // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_; // logit of occupancy probability
  double min_ray_length_;                                                                   // range of doing raycasting
  double fading_time_;

  /* visualization and computation time display */
  bool show_occ_time_;
};

// intermediate mapping data for fusion

struct MappingData
{
  Eigen::Vector3i center_last3i_;
  // Eigen::Vector3d ringbuffer_origin3d_;
  Eigen::Vector3i ringbuffer_origin3i_;
  // Eigen::Vector3d ringbuffer_division3d_;
  // Eigen::Vector3i ringbuffer_division3i_;
  Eigen::Vector3d ringbuffer_lowbound3d_;
  Eigen::Vector3i ringbuffer_lowbound3i_;
  Eigen::Vector3d ringbuffer_upbound3d_;
  Eigen::Vector3i ringbuffer_upbound3i_;
  // Eigen::Vector3d ringbuffer_size3d_;
  Eigen::Vector3i ringbuffer_size3i_;
  Eigen::Vector3i ringbuffer_inf_origin3i_;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_;
  Eigen::Vector3d ringbuffer_inf_upbound3d_;
  Eigen::Vector3i ringbuffer_inf_upbound3i_;
  Eigen::Vector3i ringbuffer_inf_size3i_;

  // main map data, occupancy of each voxel

  std::vector<double> occupancy_buffer_;
  std::vector<uint16_t> occupancy_buffer_inflate_;

  // camera position and pose data

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
  Eigen::Matrix4d cam2body_;

  // depth image data

  cv::Mat depth_image_, last_depth_image_;

  // flags of map state

  bool occ_need_update_, local_updated_;
  bool has_first_depth_;
  bool has_odom_;

  // odom_depth_timeout_
  ros::Time last_occ_update_time_;
  bool flag_depth_odom_timeout_;
  bool flag_have_ever_received_depth_;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt_;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;

  vector<Eigen::Vector3i> cache_voxel_;
  int cache_voxel_cnt_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  GridMap() {}
  ~GridMap() {}

  void initMap(ros::NodeHandle &nh);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getInflateOccupancy(Eigen::Vector3d pos);
  inline double getResolution();
  bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };

  inline Eigen::Vector3d globalIdx2Pos(const Eigen::Vector3i &id);  // 1.69ns
  inline Eigen::Vector3i pos2GlobalIdx(const Eigen::Vector3d &pos); // 0.13ns
  inline int globalIdx2BufIdx(const Eigen::Vector3i &id);           // 2.2ns
  inline int globalIdx2InfBufIdx(const Eigen::Vector3i &id);        // 2.2ns
  inline Eigen::Vector3i BufIdx2GlobalIdx(size_t address);          // 10.18ns
  inline Eigen::Vector3i infBufIdx2GlobalIdx(size_t address);       // 10.18ns
  inline bool isInBuf(const Eigen::Vector3d &pos);
  inline bool isInBuf(const Eigen::Vector3i &idx);
  inline bool isInInfBuf(const Eigen::Vector3d &pos);
  inline bool isInInfBuf(const Eigen::Vector3i &idx);

  void publishMap();
  void publishMapInflate();

  // get depth image and camera pose
  void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void extrinsicCallback(const nav_msgs::OdometryConstPtr &odom);
  void depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img);
  void odomCallback(const nav_msgs::OdometryConstPtr &odom);

  // update occupancy by raycasting
  void updateOccupancyCallback(const ros::TimerEvent & /*event*/);
  void visCallback(const ros::TimerEvent & /*event*/);
  void fadingCallback(const ros::TimerEvent & /*event*/);

  void clearBuffer(char casein, int bound);

  // main update process
  void moveRingBuffer();
  void projectDepthImage();
  void raycastProcess();
  void clearAndInflateLocalMap();

  inline void changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx);
  inline int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);
  void testIndexingCost();
  bool checkDepthOdomNeedupdate();
  void initMapBoundary();

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  ros::Subscriber indep_cloud_sub_, indep_odom_sub_, extrinsic_sub_;
  ros::Publisher map_pub_, map_inf_pub_;
  ros::Timer occ_timer_, vis_timer_, fading_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id = pos2GlobalIdx(pos);
  int idx_ctns = globalIdx2BufIdx(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_[md_.cache_voxel_cnt_++] = id;
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

inline void GridMap::changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx)
{
  int inf_grid = mp_.inf_grid_;
  if (dir)
    md_.occupancy_buffer_inflate_[inf_buf_idx] += GRID_MAP_OBS_FLAG;
  else
    md_.occupancy_buffer_inflate_[inf_buf_idx] -= GRID_MAP_OBS_FLAG;

  for (int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf)
    for (int y_inf = -inf_grid; y_inf <= inf_grid; ++y_inf)
      for (int z_inf = -inf_grid; z_inf <= inf_grid; ++z_inf)
      {
        Eigen::Vector3i id_inf(global_idx + Eigen::Vector3i(x_inf, y_inf, z_inf));
#if GRID_MAP_NEW_PLATFORM_TEST
        if (isInInfBuf(id_inf))
        {
          int id_inf_buf = globalIdx2InfBufIdx(id_inf);
          if (dir)
            ++md_.occupancy_buffer_inflate_[id_inf_buf];
          else
          {
            --md_.occupancy_buffer_inflate_[id_inf_buf];
            if (md_.occupancy_buffer_inflate_[id_inf_buf] > 65000) // An error case
            {
              ROS_ERROR("A negtive value of nearby obstacle number! reset the map.");
              fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_);
              fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0L);
            }
          }

          if (md_.occupancy_buffer_inflate_[id_inf_buf] > 60000)
          {
            cout << "2 occ=" << md_.occupancy_buffer_inflate_[id_inf_buf] << " id_inf_buf=" << id_inf_buf << " id_inf=" << id_inf.transpose() << " pos=" << globalIdx2Pos(id_inf).transpose() << endl;
          }
        }
        else
        {
          cout << "id_inf=" << id_inf.transpose() << " md_.ringbuffer_inf_upbound3i_=" << md_.ringbuffer_inf_upbound3i_.transpose() << " md_.ringbuffer_upbound3i_=" << md_.ringbuffer_upbound3i_.transpose() << endl;
          ROS_ERROR("isInInfBuf return false 1");
        }

#else
        int id_inf_buf = globalIdx2InfBufIdx(id_inf);
        if (dir)
          ++md_.occupancy_buffer_inflate_[id_inf_buf];
        else
        {
          --md_.occupancy_buffer_inflate_[id_inf_buf];
          if (md_.occupancy_buffer_inflate_[id_inf_buf] > 65000) // An error case
          {
            ROS_ERROR("A negtive value of nearby obstacle number! reset the map.");
            fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_);
            fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0L);
          }
        }
#endif
      }
}

inline int GridMap::globalIdx2BufIdx(const Eigen::Vector3i &id)
{
  int x_buffer = (id(0) - md_.ringbuffer_origin3i_(0)) % md_.ringbuffer_size3i_(0);
  int y_buffer = (id(1) - md_.ringbuffer_origin3i_(1)) % md_.ringbuffer_size3i_(1);
  int z_buffer = (id(2) - md_.ringbuffer_origin3i_(2)) % md_.ringbuffer_size3i_(2);
  if (x_buffer < 0)
    x_buffer += md_.ringbuffer_size3i_(0);
  if (y_buffer < 0)
    y_buffer += md_.ringbuffer_size3i_(1);
  if (z_buffer < 0)
    z_buffer += md_.ringbuffer_size3i_(2);

  return md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1) * z_buffer + md_.ringbuffer_size3i_(0) * y_buffer + x_buffer;
}

inline int GridMap::globalIdx2InfBufIdx(const Eigen::Vector3i &id)
{
  int x_buffer = (id(0) - md_.ringbuffer_inf_origin3i_(0)) % md_.ringbuffer_inf_size3i_(0);
  int y_buffer = (id(1) - md_.ringbuffer_inf_origin3i_(1)) % md_.ringbuffer_inf_size3i_(1);
  int z_buffer = (id(2) - md_.ringbuffer_inf_origin3i_(2)) % md_.ringbuffer_inf_size3i_(2);
  if (x_buffer < 0)
    x_buffer += md_.ringbuffer_inf_size3i_(0);
  if (y_buffer < 0)
    y_buffer += md_.ringbuffer_inf_size3i_(1);
  if (z_buffer < 0)
    z_buffer += md_.ringbuffer_inf_size3i_(2);

  return md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1) * z_buffer + md_.ringbuffer_inf_size3i_(0) * y_buffer + x_buffer;
}

inline Eigen::Vector3i GridMap::BufIdx2GlobalIdx(size_t address)
{

  const int ringbuffer_xysize = md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1);
  int zid_in_buffer = address / ringbuffer_xysize;
  address %= ringbuffer_xysize;
  int yid_in_buffer = address / md_.ringbuffer_size3i_(0);
  int xid_in_buffer = address % md_.ringbuffer_size3i_(0);

  int xid_global = xid_in_buffer + md_.ringbuffer_origin3i_(0);
  if (xid_global > md_.ringbuffer_upbound3i_(0))
    xid_global -= md_.ringbuffer_size3i_(0);
  int yid_global = yid_in_buffer + md_.ringbuffer_origin3i_(1);
  if (yid_global > md_.ringbuffer_upbound3i_(1))
    yid_global -= md_.ringbuffer_size3i_(1);
  int zid_global = zid_in_buffer + md_.ringbuffer_origin3i_(2);
  if (zid_global > md_.ringbuffer_upbound3i_(2))
    zid_global -= md_.ringbuffer_size3i_(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

inline Eigen::Vector3i GridMap::infBufIdx2GlobalIdx(size_t address)
{

  const int ringbuffer_xysize = md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1);
  int zid_in_buffer = address / ringbuffer_xysize;
  address %= ringbuffer_xysize;
  int yid_in_buffer = address / md_.ringbuffer_inf_size3i_(0);
  int xid_in_buffer = address % md_.ringbuffer_inf_size3i_(0);

  int xid_global = xid_in_buffer + md_.ringbuffer_inf_origin3i_(0);
  if (xid_global > md_.ringbuffer_inf_upbound3i_(0))
    xid_global -= md_.ringbuffer_inf_size3i_(0);
  int yid_global = yid_in_buffer + md_.ringbuffer_inf_origin3i_(1);
  if (yid_global > md_.ringbuffer_inf_upbound3i_(1))
    yid_global -= md_.ringbuffer_inf_size3i_(1);
  int zid_global = zid_in_buffer + md_.ringbuffer_inf_origin3i_(2);
  if (zid_global > md_.ringbuffer_inf_upbound3i_(2))
    zid_global -= md_.ringbuffer_inf_size3i_(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInBuf(pos))
    return 0;

  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;

  return md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(pos))] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  if (!isInInfBuf(pos))
    return 0;

  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;

  return int(md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);
}

inline bool GridMap::isInBuf(const Eigen::Vector3d &pos)
{
  if (pos(0) < md_.ringbuffer_lowbound3d_(0) || pos(1) < md_.ringbuffer_lowbound3d_(1) || pos(2) < md_.ringbuffer_lowbound3d_(2))
  {
    return false;
  }
  if (pos(0) > md_.ringbuffer_upbound3d_(0) || pos(1) > md_.ringbuffer_upbound3d_(1) || pos(2) > md_.ringbuffer_upbound3d_(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInBuf(const Eigen::Vector3i &idx)
{
  if (idx(0) < md_.ringbuffer_lowbound3i_(0) || idx(1) < md_.ringbuffer_lowbound3i_(1) || idx(2) < md_.ringbuffer_lowbound3i_(2))
  {
    return false;
  }
  if (idx(0) > md_.ringbuffer_upbound3i_(0) || idx(1) > md_.ringbuffer_upbound3i_(1) || idx(2) > md_.ringbuffer_upbound3i_(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3d &pos)
{
  if (pos(0) < md_.ringbuffer_inf_lowbound3d_(0) || pos(1) < md_.ringbuffer_inf_lowbound3d_(1) || pos(2) < md_.ringbuffer_inf_lowbound3d_(2))
  {
    return false;
  }
  if (pos(0) > md_.ringbuffer_inf_upbound3d_(0) || pos(1) > md_.ringbuffer_inf_upbound3d_(1) || pos(2) > md_.ringbuffer_inf_upbound3d_(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3i &idx)
{
  if (idx(0) < md_.ringbuffer_inf_lowbound3i_(0) || idx(1) < md_.ringbuffer_inf_lowbound3i_(1) || idx(2) < md_.ringbuffer_inf_lowbound3i_(2))
  {
    return false;
  }
  if (idx(0) > md_.ringbuffer_inf_upbound3i_(0) || idx(1) > md_.ringbuffer_inf_upbound3i_(1) || idx(2) > md_.ringbuffer_inf_upbound3i_(2))
  {
    return false;
  }
  return true;
}

inline Eigen::Vector3d GridMap::globalIdx2Pos(const Eigen::Vector3i &id) // t ~ 0us
{
  return Eigen::Vector3d((id(0) + 0.5) * mp_.resolution_, (id(1) + 0.5) * mp_.resolution_, (id(2) + 0.5) * mp_.resolution_);
}

inline Eigen::Vector3i GridMap::pos2GlobalIdx(const Eigen::Vector3d &pos)
{
  return (pos * mp_.resolution_inv_).array().floor().cast<int>(); // more than twice faster than std::floor()
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif