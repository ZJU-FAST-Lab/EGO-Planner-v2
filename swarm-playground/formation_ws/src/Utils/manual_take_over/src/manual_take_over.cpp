#include <Eigen/Eigen>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <std_msgs/Empty.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/Joy.h>


using namespace std;

ros::Publisher mandatory_stop_pub_, cmd_pub_;

ros::Time recv_joy_time_;
bool flag_recv_joy_ = false;

ros::Time recv_cmd_time_;
bool flag_recv_cmd_ = false;
bool flag_planner_stop_cmds_ = false;

bool flag_mandatory_stoped_ = false;

Eigen::Vector4d cur_drone_pos;

sensor_msgs::Joy joy_;

void joy_sub_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  ros::Time now = ros::Time::now();
  flag_recv_joy_ = true;
  recv_joy_time_ = now;
  joy_ = *msg;

  if ( msg->buttons[0] || msg->buttons[1] || msg->buttons[2] || msg->buttons[3] )
  {
    flag_mandatory_stoped_ = true;
    std_msgs::Empty stop_msg;
    mandatory_stop_pub_.publish(stop_msg);
  }

  if ( flag_mandatory_stoped_ && flag_planner_stop_cmds_ )
  {
    constexpr double MAX_VEL = 0.2;
    static bool have_last_cmd = false;
    static ros::Time last_cmd_t;
    static Eigen::Vector4d last_cmd;

    if ( !have_last_cmd )
    {
      have_last_cmd = true;
      last_cmd_t = now;
      last_cmd = cur_drone_pos;
    }

    quadrotor_msgs::PositionCommand cmd_msg;
    cmd_msg.header.stamp = now;
    cmd_msg.header.frame_id = "manual_take_over";
    double delta_t = (now - last_cmd_t).toSec();
    last_cmd(0) += joy_.axes[4] * MAX_VEL * delta_t;
    last_cmd(1) += joy_.axes[3] * MAX_VEL * delta_t;
    last_cmd(2) += joy_.axes[1] * MAX_VEL * delta_t;
    last_cmd(3) += joy_.axes[0] * MAX_VEL * delta_t;
    cmd_msg.position.x = last_cmd(0);
    cmd_msg.position.y = last_cmd(1); 
    cmd_msg.position.z = last_cmd(2); 
    cmd_msg.yaw = last_cmd(3); 
    cmd_msg.velocity.x = joy_.axes[4] * MAX_VEL;
    cmd_msg.velocity.y = joy_.axes[3] * MAX_VEL; 
    cmd_msg.velocity.z = joy_.axes[1] * MAX_VEL; 
    cmd_pub_.publish(cmd_msg);

    last_cmd_t = now;
  }
  
}

// #      ^                ^
// #    +1|              +4|
// # <-+0      ->     <-+3      ->        
// #      |                |
// #      V                V

void position_cmd_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  if ( msg->header.frame_id != string("manual_take_over") )
  {
    flag_recv_cmd_ = true;
    recv_cmd_time_ = ros::Time::now();
  }

  cur_drone_pos(0) = msg->position.x;
  cur_drone_pos(1) = msg->position.y;
  cur_drone_pos(2) = msg->position.z;
  cur_drone_pos(3) = msg->yaw;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manual_take_over");
  ros::NodeHandle nh("~");

  mandatory_stop_pub_ = nh.advertise<std_msgs::Empty>("/mandatory_stop_to_planner", 10);
  cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joystick_from_bridge", 10, joy_sub_cb);
  ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd", 100, position_cmd_sub_cb);

  while (ros::ok())
  {
    ros::Time t_now = ros::Time::now();
    
    constexpr double TIME_OUT = 1.0;
    if ( flag_recv_joy_ && (t_now - recv_joy_time_).toSec() > TIME_OUT )
    {
      ROS_ERROR("Lost manual take over joystick messages!");
    }

    if ( flag_recv_cmd_ && (t_now - recv_cmd_time_).toSec() > TIME_OUT )
    {
      flag_planner_stop_cmds_ = true;
    }
    else
    {
      flag_planner_stop_cmds_ = false;
    }
    

    ros::Duration(0.4).sleep();
    ros::spinOnce();
  }

  return 0;
}
