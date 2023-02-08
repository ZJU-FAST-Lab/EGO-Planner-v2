#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <optimizer/poly_traj_utils.hpp>

using std::vector;

namespace ego_planner
{

  typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

  struct GlobalTrajData
  {
    poly_traj::Trajectory traj;
    double global_start_time; // world time
    double duration;

    /* Global traj time. 
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double glb_t_of_lc_tgt;
    /* Global traj time. 
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double last_glb_t_of_lc_tgt;
  };

  struct LocalTrajData
  {
    poly_traj::Trajectory traj;
    PtsChk_t pts_chk;
    int drone_id; // A negative value indicates no received trajectories.
    int traj_id;
    double duration;
    double start_time; // world time
    double end_time;   // world time
    Eigen::Vector3d start_pos;
    double des_clearance;

  };

  typedef std::vector<LocalTrajData> SwarmTrajData;

  class TrajContainer
  {
  public:
    GlobalTrajData global_traj;
    LocalTrajData local_traj;
    SwarmTrajData swarm_traj;

    TrajContainer()
    {
      local_traj.traj_id = 0;
    }
    ~TrajContainer() {}

    void setGlobalTraj(const poly_traj::Trajectory &trajectory, const double &world_time)
    {
      global_traj.traj = trajectory;
      global_traj.duration = trajectory.getTotalDuration();
      global_traj.global_start_time = world_time;
      global_traj.glb_t_of_lc_tgt = world_time;
      global_traj.last_glb_t_of_lc_tgt = -1.0;

      local_traj.drone_id = -1;
      local_traj.duration = 0.0;
      local_traj.traj_id = 0;
    }

    void setLocalTraj(const poly_traj::Trajectory &trajectory, const PtsChk_t &pts_to_chk, const double &world_time, const int drone_id = -1)
    {
      local_traj.drone_id = drone_id;
      local_traj.traj_id++;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.traj = trajectory;
      local_traj.pts_chk = pts_to_chk;
    }

  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_;     // physical limits
    double polyTraj_piece_length;  // distance between adjacient B-spline control points
    double feasibility_tolerance_; // permitted ratio of vel/acc exceeding limits
    double planning_horizen_;
    bool use_multitopology_trajs;
    bool touch_goal;
    int drone_id; // single drone: drone_id <= -1, swarm: drone_id >= 0

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

} // namespace ego_planner

#endif