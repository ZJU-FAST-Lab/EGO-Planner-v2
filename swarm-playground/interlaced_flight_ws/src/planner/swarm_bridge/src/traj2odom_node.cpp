#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <traj_utils/MINCOTraj.h>
#include <optimizer/poly_traj_utils.hpp>

using namespace std;

struct Traj_t
{
  poly_traj::Trajectory traj;
  bool valid;
  ros::Time start_time;
  double duration;
  double last_yaw;
};

vector<Traj_t> trajs_;
ros::Subscriber one_traj_sub_;
ros::Publisher other_odoms_pub_;

void one_traj_sub_cb(const traj_utils::MINCOTrajPtr &msg)
{

  const int recv_id = msg->drone_id;

  if (msg->drone_id < 0)
  {
    ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
    return;
  }
  if (msg->order != 5)
  {
    ROS_ERROR("Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() != (msg->inner_x.size() + 1))
  {
    ROS_ERROR("WRONG trajectory parameters.");
    return;
  }
  if ((int)trajs_.size() > recv_id &&
      (msg->start_time - trajs_[recv_id].start_time).toSec() <= 0)
  {
    ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
    return;
  }

  ros::Time t_now = ros::Time::now();
  if (abs((t_now - msg->start_time).toSec()) > 0.25)
  {
    if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
    {
      ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
               msg->drone_id, (t_now - msg->start_time).toSec());
    }
    else
    {
      ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                msg->drone_id, (t_now - msg->start_time).toSec());
      return;
    }
  }

  /* Fill up the buffer */
  if ((int)trajs_.size() <= recv_id)
  {
    for (int i = trajs_.size(); i <= recv_id; i++)
    {
      Traj_t blank;
      blank.valid = false;
      trajs_.push_back(blank);
    }
  }

  /* Store data */;
  int piece_nums = msg->duration.size();
  Eigen::Matrix<double, 3, 3> headState, tailState;
  headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
      msg->start_p[1], msg->start_v[1], msg->start_a[1],
      msg->start_p[2], msg->start_v[2], msg->start_a[2];
  tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
      msg->end_p[1], msg->end_v[1], msg->end_a[1],
      msg->end_p[2], msg->end_v[2], msg->end_a[2];
  Eigen::MatrixXd innerPts(3, piece_nums - 1);
  Eigen::VectorXd durations(piece_nums);
  for (int i = 0; i < piece_nums - 1; i++)
    innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
  for (int i = 0; i < piece_nums; i++)
    durations(i) = msg->duration[i];
  poly_traj::MinJerkOpt MJO;
  MJO.reset(headState, tailState, piece_nums);
  MJO.generate(innerPts, durations);

  trajs_[recv_id].traj = MJO.getTraj();
  trajs_[recv_id].start_time = msg->start_time;
  trajs_[recv_id].valid = true;
  trajs_[recv_id].duration = trajs_[recv_id].traj.getTotalDuration();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj2odom");
  ros::NodeHandle nh("~");

  double odom_hz;
  nh.param("odom_hz", odom_hz, 100.0);

  one_traj_sub_ = nh.subscribe("/broadcast_traj_to_planner", 100, one_traj_sub_cb, ros::TransportHints().tcpNoDelay());
  other_odoms_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom", 100);

  ros::Rate loop_rate(odom_hz);
  while (ros::ok())
  {
    auto t_now = ros::Time::now();

    for (int id = 0; id < (int)trajs_.size(); ++id)
    {
      if (trajs_[id].valid)
      {
        double t_to_start = (t_now - trajs_[id].start_time).toSec();
        if (t_to_start <= trajs_[id].duration)
        {
          double t = t_to_start;
          Eigen::Vector3d p = trajs_[id].traj.getPos(t);
          Eigen::Vector3d v = trajs_[id].traj.getVel(t);
          double yaw = v.head(2).norm() > 0.01 ? atan2(v(1), v(0)) : trajs_[id].last_yaw; //
          trajs_[id].last_yaw = yaw;
          Eigen::AngleAxisd rotation_vector(yaw, Eigen::Vector3d::UnitZ());
          Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);

          nav_msgs::Odometry msg;
          msg.header.frame_id = string("world");
          msg.header.stamp = t_now;
          msg.child_frame_id = string("drone_") + to_string(id);
          msg.pose.pose.position.x = p(0);
          msg.pose.pose.position.y = p(1);
          msg.pose.pose.position.z = p(2);
          msg.pose.pose.orientation.w = q.w();
          msg.pose.pose.orientation.x = q.x();
          msg.pose.pose.orientation.y = q.y();
          msg.pose.pose.orientation.z = q.z();
          msg.twist.twist.linear.x = v(0);
          msg.twist.twist.linear.y = v(1);
          msg.twist.twist.linear.z = v(2);

          other_odoms_pub_.publish(msg);
        }
      }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}