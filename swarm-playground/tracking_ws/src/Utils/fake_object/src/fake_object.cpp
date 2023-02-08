#include <Eigen/Eigen>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

using namespace std;

ros::Publisher obj_odom_pub_;

// sensor_msgs::Joy joy_;

constexpr double INIT_X = 0;
constexpr double INIT_Y = 0;
constexpr double INIT_Z = 1.0;
double x_, y_, z_;
double last_x_, last_y_, last_z_;
ros::Time last_t_;

void joy_sub_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  // ros::Time now = ros::Time::now();
  // joy_ = *msg;

  x_ += msg->axes[4] / 8;
  y_ += msg->axes[3] / 8;
  z_ += msg->axes[1] / 8;

  ros::Time t_now = ros::Time::now();
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = t_now;
  odom_msg.header.frame_id = "world";
  odom_msg.pose.pose.position.x = x_ + (((double)rand() / RAND_MAX) - 0.5) / 5;
  odom_msg.pose.pose.position.y = y_ + (((double)rand() / RAND_MAX) - 0.5) / 5;
  // odom_msg.pose.pose.position.z = z_ + (((double)rand() / RAND_MAX) - 0.5) / 10;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.twist.twist.linear.x = (x_ + (((double)rand() / RAND_MAX) - 0.5) / 5 - last_x_) / (t_now - last_t_).toSec();
  odom_msg.twist.twist.linear.y = (y_ + (((double)rand() / RAND_MAX) - 0.5) / 5 - last_y_) / (t_now - last_t_).toSec();
  // odom_msg.twist.twist.linear.z = (z_ + (((double)rand() / RAND_MAX) - 0.5) / 10 - last_z_) / (t_now - last_t_).toSec();
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.pose.pose.orientation.w = 1.0;
  obj_odom_pub_.publish(odom_msg);

  last_x_ = x_;
  last_y_ = y_;
  last_z_ = z_;
  last_t_ = t_now;

  if (msg->buttons[0] || msg->buttons[1] || msg->buttons[2] || msg->buttons[3])
  {
    last_x_ = INIT_X;
    last_y_ = INIT_Y;
    last_z_ = INIT_Z;
  }
}

// #      ^                ^
// #    +1|              +4|
// # <-+0      ->     <-+3      ->
// #      |                |
// #      V                V

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_object");
  ros::NodeHandle nh("~");

  obj_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/object_odom", 10);
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, joy_sub_cb);

  x_ = INIT_X;
  y_ = INIT_Y;
  z_ = INIT_Z;
  last_x_ = INIT_X;
  last_y_ = INIT_Y;
  last_z_ = INIT_Z;
  last_t_ = ros::Time::now();

  while (ros::ok())
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  return 0;
}
