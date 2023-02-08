#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <traj_utils/MINCOTraj.h>
#include <quadrotor_msgs/GoalSet.h>
#include <sensor_msgs/Joy.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#define UDP_PORT 8081
#define BUF_LEN 1048576    // 1MB
#define BUF_LEN_SHORT 1024 // 1KB

using namespace std;

int udp_server_fd_, udp_send_fd_;
ros::Subscriber other_odoms_sub_, one_traj_sub_, mandatory_stop_sub_, goal_sub_, joy_sub_;
ros::Publisher other_odoms_pub_, one_traj_pub_, mandatory_stop_pub_, goal_pub_, joy_pub_;
string udp_ip_;
int drone_id_;
double odom_broadcast_freq_;
char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;
nav_msgs::Odometry odom_msg_;
traj_utils::MINCOTraj MINCOTraj_msg_;
std_msgs::Empty stop_msg_;
quadrotor_msgs::GoalSet goal_msg_;
sensor_msgs::Joy joy_msg_;

enum MESSAGE_TYPE
{
  ODOM = 100,
  ONE_TRAJ,
  STOP,
  GOAL,
  JOY
} massage_type_;

int init_broadcast(const char *ip, const int port)
{
  int fd;

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
  {
    ROS_ERROR("[bridge_node]Socket sender creation error!");
    exit(EXIT_FAILURE);
  }

  int so_broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
  {
    cout << "Error in setting Broadcast option";
    exit(EXIT_FAILURE);
  }

  addr_udp_send_.sin_family = AF_INET;
  addr_udp_send_.sin_port = htons(port);

  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
  {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  return fd;
}

int udp_bind_to_port(const int port, int &server_fd)
{
  struct sockaddr_in address;
  int opt = 1;

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                 &opt, sizeof(opt)))
  {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  // Forcefully attaching socket to the port
  if (bind(server_fd, (struct sockaddr *)&address,
           sizeof(address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  return server_fd;
}

template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg)
{
  auto ptr = (uint8_t *)(udp_send_buf_);

  *((MESSAGE_TYPE*)ptr) = msg_type;
  ptr += sizeof(MESSAGE_TYPE);

  namespace ser = ros::serialization;
  uint32_t msg_size = ser::serializationLength(msg);

  *((uint32_t *)ptr) = msg_size;
  ptr += sizeof(uint32_t);

  ser::OStream stream(ptr, msg_size);
  ser::serialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

template <typename T>
int deserializeTopic(T &msg)
{
  auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));

  uint32_t msg_size = *((uint32_t *)ptr);
  ptr += sizeof(uint32_t);

  namespace ser = ros::serialization;
  ser::IStream stream(ptr, msg_size);
  ser::deserialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

void odom_sub_udp_cb(const nav_msgs::OdometryPtr &msg)
{

  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  msg->child_frame_id = string("drone_") + std::to_string(drone_id_);

  int len = serializeTopic(MESSAGE_TYPE::ODOM, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (1)!!!");
  }
}

void one_traj_sub_udp_cb(const traj_utils::MINCOTrajPtr &msg)
{

  int len = serializeTopic(MESSAGE_TYPE::ONE_TRAJ, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (2)!!!");
  }
}

void mandatory_stop_sub_udp_cb(const std_msgs::EmptyPtr &msg)
{

  int len = serializeTopic(MESSAGE_TYPE::STOP, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (3)!!!");
  }
}

void goal_sub_udp_cb(const quadrotor_msgs::GoalSetPtr &msg)
{

  int len = serializeTopic(MESSAGE_TYPE::GOAL, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (4)!!!");
  }
}

void joy_sub_udp_cb(const sensor_msgs::JoyPtr &msg)
{

  int len = serializeTopic(MESSAGE_TYPE::JOY, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (5)!!!");
  }
}

void udp_recv_fun()
{
  int valread;
  struct sockaddr_in addr_client;
  socklen_t addr_len;

  // Connect
  if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
  {
    ROS_ERROR("[bridge_node]Socket recever creation error!");
    exit(EXIT_FAILURE);
  }

  while (true)
  {
    if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0, (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0)
    {
      perror("recvfrom() < 0, error:");
      exit(EXIT_FAILURE);
    }

    char *ptr = udp_recv_buf_;
    switch (*((MESSAGE_TYPE *)ptr))
    {

    case MESSAGE_TYPE::ODOM:
    {
      if (valread == deserializeTopic(odom_msg_))
      {
        other_odoms_pub_.publish(odom_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (2)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::ONE_TRAJ:
    {

      if (valread == deserializeTopic(MINCOTraj_msg_))
      {
        one_traj_pub_.publish(MINCOTraj_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (2)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::STOP:
    {

      if (valread == deserializeTopic(stop_msg_))
      {
        mandatory_stop_pub_.publish(stop_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (3)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::GOAL:
    {
      
      if (valread == deserializeTopic(goal_msg_))
      {
        goal_pub_.publish(goal_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (4)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::JOY:
    {
      
      if (valread == deserializeTopic(joy_msg_))
      {
        joy_pub_.publish(joy_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (5)!!!");
        continue;
      }

      break;
    }

    default:

      ROS_ERROR("Unknown received message type???");

      break;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  nh.param("broadcast_ip", udp_ip_, string("127.0.0.255"));
  nh.param("drone_id", drone_id_, -1);
  nh.param("odom_max_freq", odom_broadcast_freq_, 1000.0);

  if (drone_id_ == -1)
  {
    ROS_WARN("[swarm bridge] Wrong drone_id!");
    exit(EXIT_FAILURE);
  }

  other_odoms_sub_ = nh.subscribe("my_odom", 10, odom_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  other_odoms_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom", 10);

  one_traj_sub_ = nh.subscribe("/broadcast_traj_from_planner", 100, one_traj_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  one_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/broadcast_traj_to_planner", 100);

  // mandatory_stop_sub_ = nh.subscribe("/mandatory_stop_from_users", 100, mandatory_stop_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  // mandatory_stop_pub_ = nh.advertise<std_msgs::Empty>("/mandatory_stop_to_planner", 100);

  goal_sub_ = nh.subscribe("/goal_user2brig", 100, goal_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_brig2plner", 100);

  joy_sub_ = nh.subscribe("/joystick_from_users", 100, joy_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  joy_pub_ = nh.advertise<sensor_msgs::Joy>("/joystick_from_bridge", 100);

  boost::thread udp_recv_thd(udp_recv_fun);
  udp_recv_thd.detach();
  ros::Duration(0.1).sleep();

  // UDP connect
  udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

  cout << "[rosmsg_tcp_bridge] start running" << endl;

  ros::spin();

  close(udp_server_fd_);
  close(udp_send_fd_);

  return 0;
}
