#include <Eigen/Eigen>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <traj_utils/MINCOTraj.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>

using namespace std;

ros::Publisher obs1_odom_pub_, obs2_odom_pub_, traj_pub_, predicted_traj_pub_;

double obs1_id_, obs2_id_;

ego_planner::PlanningVisualization::Ptr visualization_;

class moving_obstacle
{
private:
  Eigen::Vector2d pos_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d vel_{Eigen::Vector2d::Zero()};
  double yaw_{0};
  ros::Time t_last_update_{ros::Time(0)};

public:

  double des_clearance_;

  moving_obstacle(){};
  ~moving_obstacle(){};

  void set_position(Eigen::Vector2d pos)
  {
    pos_ = pos;
  }

  double get_yaw() { return yaw_; }

  void dyn_update(const double delta_t, const double acc, const double dir, double &yaw, Eigen::Vector2d &pos, Eigen::Vector2d &vel) const
  {
    Eigen::Vector2d acc_vec = acc * Eigen::Vector2d(cos(yaw), sin(yaw));
    vel += acc_vec * delta_t;
    vel *= 0.9; // gradually stop like a real obstacle
    constexpr double MAX_VEL = 2.0;
    if (vel.norm() > MAX_VEL)
    {
      vel /= vel.norm() / MAX_VEL;
    }
    pos += vel * delta_t + 0.5 * acc_vec * delta_t * delta_t;
    yaw += dir * delta_t;
  }

  std::pair<Eigen::Vector2d, Eigen::Vector2d> update(const double acc, const double dir)
  {
    ros::Time t_now = ros::Time::now();
    if (t_last_update_ == ros::Time(0))
    {
      t_last_update_ = t_now;
    }

    double delta_t = (t_now - t_last_update_).toSec();
    dyn_update(delta_t, acc, dir, yaw_, pos_, vel_);

    t_last_update_ = t_now;

    return std::pair<Eigen::Vector2d, Eigen::Vector2d>(pos_, vel_);
  }

  std::pair<Eigen::Vector2d, Eigen::Vector2d> predict(const double acc, const double dir, double predict_t) const
  {
    constexpr double STEP = 0.1;
    double yaw = yaw_;
    Eigen::Vector2d pos = pos_;
    Eigen::Vector2d vel = vel_;

    for (double t = STEP; t <= predict_t; t += STEP)
    {
      dyn_update(STEP, acc, dir, yaw, pos, vel);
    }

    return std::pair<Eigen::Vector2d, Eigen::Vector2d>(pos, vel);
  }
};

moving_obstacle obs1_, obs2_;

poly_traj::Trajectory predict_traj(const double acc, const double dir, const Eigen::Vector3d p, const Eigen::Vector3d v, const moving_obstacle &obstacle, vector<Eigen::Vector3d> &vis_pts)
{
  vis_pts.clear();
  constexpr double PRED_TIME = 5.0;
  constexpr int SEG_NUM = 10;
  poly_traj::MinJerkOpt predicted_traj;
  Eigen::Matrix<double, 3, 3> headState, tailState;
  headState << p, v, Eigen::Vector3d::Zero();
  Eigen::MatrixXd innerPts(3, SEG_NUM - 1);
  Eigen::VectorXd ts(SEG_NUM);
  vis_pts.push_back(headState.col(0));
  for (int i = 1; i < SEG_NUM; ++i)
  {
    auto pred_pv = obstacle.predict(acc, dir, PRED_TIME / SEG_NUM * i);
    innerPts.col(i - 1) = Eigen::Vector3d(pred_pv.first(0), pred_pv.first(1), p(2));
    ts(i - 1) = PRED_TIME / SEG_NUM;
    vis_pts.push_back(innerPts.col(i - 1));
  }
  ts(SEG_NUM - 1) = PRED_TIME / SEG_NUM;
  auto tail_pv = obstacle.predict(acc, dir, PRED_TIME);
  tailState << Eigen::Vector3d(tail_pv.first(0), tail_pv.first(1), p(2)), Eigen::Vector3d(tail_pv.second(0), tail_pv.second(1), v(2)), Eigen::Vector3d::Zero();
  vis_pts.push_back(tailState.col(0));
  predicted_traj.reset(headState, tailState, SEG_NUM);
  predicted_traj.generate(innerPts, ts);
  return predicted_traj.getTraj();
}

void Traj2ROSMsg(const poly_traj::Trajectory &traj, const double des_clear, const int obstacle_id, traj_utils::MINCOTraj &MINCO_msg)
{

  Eigen::VectorXd durs = traj.getDurations();
  int piece_num = traj.getPieceNum();
  double duration = durs.sum();

  MINCO_msg.drone_id = obstacle_id;
  MINCO_msg.traj_id = 0;
  MINCO_msg.start_time = ros::Time::now();
  MINCO_msg.order = 5; // todo, only support order = 5 now.
  MINCO_msg.duration.resize(piece_num);
  MINCO_msg.des_clearance = des_clear;
  Eigen::Vector3d vec;
  vec = traj.getPos(0);
  MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
  vec = traj.getVel(0);
  MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
  vec = traj.getAcc(0);
  MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
  vec = traj.getPos(duration);
  MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
  vec = traj.getVel(duration);
  MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
  vec = traj.getAcc(duration);
  MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
  MINCO_msg.inner_x.resize(piece_num - 1);
  MINCO_msg.inner_y.resize(piece_num - 1);
  MINCO_msg.inner_z.resize(piece_num - 1);
  Eigen::MatrixXd pos = traj.getPositions();
  for (int i = 0; i < piece_num - 1; i++)
  {
    MINCO_msg.inner_x[i] = pos(0, i + 1);
    MINCO_msg.inner_y[i] = pos(1, i + 1);
    MINCO_msg.inner_z[i] = pos(2, i + 1);
  }
  for (int i = 0; i < piece_num; i++)
    MINCO_msg.duration[i] = durs[i];
}

// #      ^                ^
// #    +1|              +4|
// # <-+0      ->     <-+3      ->
// #      |                |
// #      V                V

void joy_sub_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  ros::Time t_now = ros::Time::now();

  double acc1 = msg->axes[1] * 2;
  double dir1 = msg->axes[0] / 3;
  double acc2 = msg->axes[4] * 2;
  double dir2 = msg->axes[3] / 3;
  if (acc1 < 0)
    dir1 = -dir1;
  if (acc2 < 0)
    dir2 = -dir2;

  auto pv1 = obs1_.update(acc1, dir1);
  auto pv2 = obs2_.update(acc2, dir2);

  constexpr double HEIGHT = 1.0;

  // publish odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = t_now;
  odom_msg.header.frame_id = "world";
  odom_msg.pose.pose.position.z = HEIGHT;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;

  Eigen::Quaterniond q1(Eigen::AngleAxisd(obs1_.get_yaw(), Eigen::Vector3d::UnitZ()));
  odom_msg.pose.pose.position.x = pv1.first(0);
  odom_msg.pose.pose.position.y = pv1.first(1);
  odom_msg.twist.twist.linear.x = pv1.second(0);
  odom_msg.twist.twist.linear.y = pv1.second(1);
  odom_msg.pose.pose.orientation.w = q1.w();
  odom_msg.pose.pose.orientation.z = q1.z();
  obs1_odom_pub_.publish(odom_msg);
  ros::Duration(0.005).sleep();

  Eigen::Quaterniond q2(Eigen::AngleAxisd(obs2_.get_yaw(), Eigen::Vector3d::UnitZ()));
  odom_msg.pose.pose.position.x = pv2.first(0);
  odom_msg.pose.pose.position.y = pv2.first(1);
  odom_msg.twist.twist.linear.x = pv2.second(0);
  odom_msg.twist.twist.linear.y = pv2.second(1);
  odom_msg.pose.pose.orientation.w = q2.w();
  odom_msg.pose.pose.orientation.z = q2.z();
  obs2_odom_pub_.publish(odom_msg);
  ros::Duration(0.005).sleep();

  // publish predicted trajectory
  traj_utils::MINCOTraj MINCO_msg;
  vector<Eigen::Vector3d> vis_pts;
  poly_traj::Trajectory traj1 = predict_traj(acc1, dir1, Eigen::Vector3d(pv1.first[0], pv1.first[1], HEIGHT), Eigen::Vector3d(pv1.second[0], pv1.second[1], 0), obs1_, vis_pts);
  Traj2ROSMsg(traj1, obs1_.des_clearance_, obs1_id_, MINCO_msg);
  predicted_traj_pub_.publish(MINCO_msg);
  ros::Duration(0.005).sleep();
  visualization_->displayInitPathList(vis_pts, 0.1, obs1_id_);
  ros::Duration(0.005).sleep();

  poly_traj::Trajectory traj2 = predict_traj(acc2, dir2, Eigen::Vector3d(pv2.first[0], pv2.first[1], HEIGHT), Eigen::Vector3d(pv2.second[0], pv2.second[1], 0), obs2_, vis_pts);
  Traj2ROSMsg(traj2, obs2_.des_clearance_, obs2_id_, MINCO_msg);
  predicted_traj_pub_.publish(MINCO_msg);
  ros::Duration(0.005).sleep();
  visualization_->displayInitPathList(vis_pts, 0.1, obs2_id_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moving_obstacles");
  ros::NodeHandle nh("~");

  obs1_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_obs1", 10);
  obs2_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_obs2", 10);
  predicted_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/broadcast_traj_to_planner", 10);
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, joy_sub_cb);

  visualization_.reset(new ego_planner::PlanningVisualization(nh));

  std::vector<double> init_pos;
  nh.getParam("obstacle1_init_pos", init_pos);
  obs1_.set_position(Eigen::Vector2d(init_pos[0], init_pos[1]));
  nh.getParam("desired_clearance1", obs1_.des_clearance_);
  nh.getParam("obstacle2_init_pos", init_pos);
  obs2_.set_position(Eigen::Vector2d(init_pos[0], init_pos[1]));
  nh.getParam("desired_clearance2", obs2_.des_clearance_);
  nh.getParam("obstacle1_id", obs1_id_);
  nh.getParam("obstacle2_id", obs2_id_);

  while (ros::ok())
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  return 0;
}
