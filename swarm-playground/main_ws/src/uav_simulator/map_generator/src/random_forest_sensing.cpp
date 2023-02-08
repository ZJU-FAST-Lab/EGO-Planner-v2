#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
// default_random_engine eng(4);
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_inf;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher click_map_pub_;
ros::Subscriber _odom_sub;

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _pub_rate;
double _min_dist;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_theta_tilt_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

void RandomMapGenerate()
{
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num; i++)
  {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++)
        {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i)
  {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void RandomMapGenerateCylinder()
{
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.5, 1.5);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num && ros::ok(); i++)
  {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((w * inf) / _resolution);
    double radius = (w * inf) / 2;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -10; t < heiNum; t++)
        {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cloudMap.points.push_back(pt_random);
          }
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i)
  {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  // for (double x = -1.5; x <= 1.5; x += 0.1)
  //   for (double y = -1.5; y <= 1.5; y += 0.1)
  //     for (double z = -0.5; z <= 3.0; z += 0.1)
  //     {
  //       pt_random.x = x;
  //       pt_random.y = y;
  //       pt_random.z = z;
  //       cloudMap.push_back(pt_random);
  //     }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void RandomInclinedColumn()
{
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.2, 0.5);

  rand_theta_ = uniform_real_distribution<double>(-3.14, 3.14);
  rand_theta_tilt_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate inclined column obs
  for (int i = 0; i < _obs_num && ros::ok(); i++)
  {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    h = rand_h(eng);
    double rand_theta = rand_theta_(eng);
    double rand_theta_tilt = rand_theta_tilt_(eng);

    int widNum = ceil((w * inf) / _resolution);
    // double radius = (w * inf) / 2;
    Eigen::AngleAxisd R_v(rand_theta_tilt, Eigen::Vector3d(cos(rand_theta),sin(rand_theta),0));
    Eigen::Matrix3d R_m = R_v.matrix();

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        int heiNum = ceil(h / _resolution);
        for (int t = -10; t < heiNum; t++)
        {
          Eigen::Vector3d temp_p(x + (r + 0.5) * _resolution + 1e-2, y + (s + 0.5) * _resolution + 1e-2, (t + 0.5) * _resolution + 1e-2);
          temp_p = R_m * (temp_p - Eigen::Vector3d(x,y,0)) + Eigen::Vector3d(x,y,0);
          
          pt_random.x = temp_p(0);
          pt_random.y = temp_p(1);
          pt_random.z = temp_p(2);
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void clickCallback(const geometry_msgs::PoseStamped &msg)
{
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double w = rand_w(eng);
  double h;
  pcl::PointXYZ pt_random;

  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  int widNum = ceil(w / _resolution);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
    {
      h = rand_h(eng);
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++)
      {
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        cloudMap.points.push_back(pt_random);
      }
    }
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_.publish(localMap_pcd);

  cloudMap.width = cloudMap.points.size();

  return;
}

int i = 0;
void pubPoints()
{
  while (ros::ok())
  {
    ros::spinOnce();
    if (_map_ok)
      break;
  }

  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);

  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 7.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("pub_rate", _pub_rate, 10.0);
  n.param("min_distance", _min_dist, 1.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Duration(0.5).sleep();

  unsigned int seed = rd();
  // unsigned int seed = 3728542744;
  cout << "seed=" << seed << endl;
  eng.seed(seed);

  // RandomMapGenerate();
  RandomMapGenerateCylinder();
  // RandomInclinedColumn();

  ros::Rate loop_rate(_pub_rate);

  while (ros::ok())
  {
    pubPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}









// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// // #include <pcl/search/kdtree.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <iostream>

// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Vector3.h>
// #include <math.h>
// #include <nav_msgs/Odometry.h>
// #include <ros/console.h>
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <Eigen/Eigen>
// #include <random>
// #include <stdlib.h>

// using namespace std;

// // pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
// pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
// vector<int> pointIdxRadiusSearch;
// vector<float> pointRadiusSquaredDistance;

// fstream fp_;

// random_device rd;
// default_random_engine eng;
// uniform_real_distribution<double> rand_x;
// uniform_real_distribution<double> rand_y;
// uniform_real_distribution<double> rand_w;
// uniform_real_distribution<double> rand_h;

// ros::Publisher _local_map_pub;
// ros::Publisher _all_map_pub;
// ros::Subscriber _odom_sub;

// vector<double> _state;

// int _obs_num;
// double _x_size, _y_size, _z_size;
// double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
// double _z_limit, _sensing_range, _resolution, _pub_rate, _init_x, _init_y;

// bool _map_ok = false;
// bool _has_odom = false;

// int circle_num_;
// double radius_l_, radius_h_, z_l_, z_h_;
// double theta_;
// uniform_real_distribution<double> rand_radius_;
// uniform_real_distribution<double> rand_radius2_;
// uniform_real_distribution<double> rand_theta_;
// uniform_real_distribution<double> rand_z_;

// sensor_msgs::PointCloud2 localMap_pcd;
// sensor_msgs::PointCloud2 globalMap_pcd;
// pcl::PointCloud<pcl::PointXYZ> cloudMap, cloudMap_buf;

// std::string _map_file_name;
// std::string _map_file_dir;

// void RandomMapGenerate()
// {
//   cloudMap.points.clear();

//   pcl::PointXYZ pt_random;

//   rand_x = uniform_real_distribution<double>(_x_l, _x_h);
//   rand_y = uniform_real_distribution<double>(_y_l, _y_h);
//   rand_w = uniform_real_distribution<double>(_w_l, _w_h);
//   rand_h = uniform_real_distribution<double>(_h_l, _h_h);

//   rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
//   rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
//   rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
//   rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

//   // generate polar obs
//   for (int i = 0; i < _obs_num; i++)
//   {
//     double x, y, w, h;
//     x = rand_x(eng);
//     y = rand_y(eng);
//     w = rand_w(eng);

//     // if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0)
//     // {
//     //   i--;
//     //   continue;
//     // }

//     // if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0)
//     // {
//     //   i--;
//     //   continue;
//     // }

//     if (sqrt(pow(x - 4, 2) + pow(y - 0, 2)) < 1.0)
//     {
//       i--;
//       continue;
//     }

//     if (sqrt(pow(x - (-4), 2) + pow(y - 0.0, 2)) < 1.0)
//     {
//       i--;
//       continue;
//     }

//     x = floor(x / _resolution) * _resolution + _resolution / 2.0;
//     y = floor(y / _resolution) * _resolution + _resolution / 2.0;

//     int widNum = ceil(w / _resolution);

//     for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
//       for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
//       {
//         h = rand_h(eng);
//         int heiNum = ceil(h / _resolution);
//         for (int t = ceil(-1 / _resolution); t < heiNum; t++)
//         {
//           pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
//           pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
//           pt_random.z = (t + 0.5) * _resolution + 1e-2;
//           cloudMap.points.push_back(pt_random);
//         }
//       }
//   }

//   // generate circle obs
//   for (int i = 0; i < circle_num_; ++i)
//   {
//     double x, y, z;
//     x = rand_x(eng);
//     y = rand_y(eng);
//     z = rand_z_(eng);

//     // if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0)
//     // {
//     //   i--;
//     //   continue;
//     // }

//     // if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0)
//     // {
//     //   i--;
//     //   continue;
//     // }

//     if (sqrt(pow(x - 4, 2) + pow(y - 0, 2)) < 1.0)
//     {
//       i--;
//       continue;
//     }

//     if (sqrt(pow(x - (-4), 2) + pow(y - 0.0, 2)) < 1.0)
//     {
//       i--;
//       continue;
//     }

//     x = floor(x / _resolution) * _resolution + _resolution / 2.0;
//     y = floor(y / _resolution) * _resolution + _resolution / 2.0;
//     z = floor(z / _resolution) * _resolution + _resolution / 2.0;

//     Eigen::Vector3d translate(x, y, z);

//     double theta = rand_theta_(eng);
//     Eigen::Matrix3d rotate;
//     rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

//     double radius1 = rand_radius_(eng);
//     double radius2 = rand_radius2_(eng);

//     // draw a circle centered at (x,y,z)
//     Eigen::Vector3d cpt;
//     for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
//     {
//       cpt(0) = 0.0;
//       cpt(1) = radius1 * cos(angle);
//       cpt(2) = radius2 * sin(angle);

//       // inflate
//       Eigen::Vector3d cpt_if;
//       for (int ifx = -1; ifx <= 1; ++ifx)
//         for (int ify = -1; ify <= 1; ++ify)
//           for (int ifz = -1; ifz <= 1; ++ifz)
//           {
//             cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution, ifz * _resolution);
//             cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
//             pt_random.x = cpt_if(0);
//             pt_random.y = cpt_if(1);
//             pt_random.z = cpt_if(2);
//             cloudMap.push_back(pt_random);
//           }
//     }
//   }

//   cloudMap.width = cloudMap.points.size();
//   cloudMap.height = 1;
//   cloudMap.is_dense = true;

//   //ROS_WARN("Finished generate random map ");

//   kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

//   _map_ok = true;
// }

// void PseudoRandomMapGenerate()
// {
//   cloudMap.points.clear();

//   pcl::PointXYZ pt_random;

//   // rand_x = uniform_real_distribution<double>(_x_l, _x_h);
//   // rand_y = uniform_real_distribution<double>(_y_l, _y_h);
//   // rand_w = uniform_real_distribution<double>(_w_l, _w_h);
//   // rand_h = uniform_real_distribution<double>(_h_l, _h_h);

//   // rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
//   // rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
//   // rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
//   // rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

//   // generate polar obs
//   for (int i = 0; i < _obs_num; i++)
//   {
//     double x, y, w, h;
//     x = (double)rand() / RAND_MAX * (_x_h - _x_l) + _x_l;
//     y = (double)rand() / RAND_MAX * (_y_h - _y_l) + _y_l;
//     w = (double)rand() / RAND_MAX * (_w_h - _w_l) + _w_l;

//     // if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0)
//     // {
//     //   i--;
//     //   continue;
//     // }

//     // if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0)
//     // {
//     //   i--;
//     //   continue;
//     // }

//     if (sqrt(pow(x - 4, 2) + pow(y - 0, 2)) < 2.0)
//     {
//       i--;
//       continue;
//     }

//     if (sqrt(pow(x - (-4), 2) + pow(y - 0.0, 2)) < 2.0)
//     {
//       i--;
//       continue;
//     }

//     x = floor(x / _resolution) * _resolution + _resolution / 2.0;
//     y = floor(y / _resolution) * _resolution + _resolution / 2.0;

//     int widNum = ceil(w / _resolution);

//     for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
//       for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
//       {
//         h = (double)rand() / RAND_MAX * (_h_h - _h_l) + _h_l;
//         int heiNum = ceil(h / _resolution);
//         for (int t = ceil(-1 / _resolution); t < heiNum; t++)
//         {
//           pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
//           pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
//           pt_random.z = (t + 0.5) * _resolution + 1e-2;
//           if (sqrt(pow(pt_random.x - x, 2) + pow(pt_random.y - y, 2)) > _w_h / 2)
//             continue;
//           cloudMap.points.push_back(pt_random);
//         }
//       }
//   }

//   // generate circle obs
//   for (int i = 0; i < circle_num_; ++i)
//   {
//     double x, y, z;
//     x = (double)rand() / RAND_MAX * (_x_h - _x_l) + _x_l;
//     y = (double)rand() / RAND_MAX * (_y_h - _y_l) + _y_l;
//     z = (double)rand() / RAND_MAX * (z_h_ - z_l_) + z_l_;

//     // if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0)
//     // {
//     //   i--;
//     //   continue;
//     // }

//     // if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0)
//     // {
//     //   i--;
//     //   continue;
//     // }

//     if (sqrt(pow(x - 4, 2) + pow(y - 0, 2)) < 1.0)
//     {
//       i--;
//       continue;
//     }

//     if (sqrt(pow(x - (-4), 2) + pow(y - 0.0, 2)) < 1.0)
//     {
//       i--;
//       continue;
//     }

//     x = floor(x / _resolution) * _resolution + _resolution / 2.0;
//     y = floor(y / _resolution) * _resolution + _resolution / 2.0;
//     z = floor(z / _resolution) * _resolution + _resolution / 2.0;

//     Eigen::Vector3d translate(x, y, z);

//     double theta = (double)rand() / RAND_MAX * (2 * theta_) - theta_;
//     Eigen::Matrix3d rotate;
//     rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

//     double radius1 = (double)rand() / RAND_MAX * (radius_h_ - radius_l_) + radius_l_;
//     ;
//     double radius2 = (double)rand() / RAND_MAX * (1.2 - radius_l_) + radius_l_;

//     // draw a circle centered at (x,y,z)
//     Eigen::Vector3d cpt;
//     for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
//     {
//       cpt(0) = 0.0;
//       cpt(1) = radius1 * cos(angle);
//       cpt(2) = radius2 * sin(angle);

//       // inflate
//       Eigen::Vector3d cpt_if;
//       for (int ifx = -1; ifx <= 1; ++ifx)
//         for (int ify = -1; ify <= 1; ++ify)
//           for (int ifz = -1; ifz <= 1; ++ifz)
//           {
//             cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution, ifz * _resolution);
//             cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
//             pt_random.x = cpt_if(0);
//             pt_random.y = cpt_if(1);
//             pt_random.z = cpt_if(2);
//             cloudMap.push_back(pt_random);
//           }
//     }
//   }

//   cloudMap.width = cloudMap.points.size();
//   cloudMap.height = 1;
//   cloudMap.is_dense = true;

//   //ROS_WARN("Finished generate random map ");

//   kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

//   _map_ok = true;
// }

// void CustomMapGenerate0(void)
// {
//   pcl::PointXYZ pt;

//   // generate polar obs
//   for (double x = -0.5; x <= 0.5; x += _resolution)
//     for (double y = -0.5; y <= 0.5; y += _resolution)
//       for (double z = 0.0; z <= 3.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   _map_ok = true;
// }

// void CustomMapGenerate1(void)
// {
//   pcl::PointXYZ pt;

//   // generate polar obs
//   for (double x = -1.2; x <= -0.8; x += _resolution)
//     for (double y = -0.5; y <= 2.0; y += _resolution)
//       for (double z = -1.0; z <= 3.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   // generate polar obs
//   for (double x = 0.8; x <= 1.2; x += _resolution)
//     for (double y = -2.0; y <= 0.5; y += _resolution)
//       for (double z = -1.0; z <= 3.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   _map_ok = true;
// }

// void CustomMapGenerateEasy(void)
// {
//   pcl::PointXYZ pt;

//   // generate polar obs
//   for (double x = -2.0; x <= -1; x += _resolution)
//     for (double y = -0.1; y <= 0.9; y += _resolution)
//       for (double z = 0.0; z <= 3.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   for (double r = 0.0; r <= 0.8; r += _resolution)
//     for (double theta = 0; theta <= 2 * 3.15; theta += 0.05)
//       for (double z = 0; z <= 3; z += _resolution)
//       {
//         pt.x = cos(theta) * r + 1.0;
//         pt.y = sin(theta) * r - 0.5;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   _map_ok = true;
// }

// void CustomMapGenerate2(void)
// {
//   pcl::PointXYZ pt;

//   // generate polar obs
//   for (double x = -1.5; x <= -0.5; x += _resolution)
//     for (double y = -0.7; y <= 0.5; y += _resolution)
//       for (double z = 0.0; z <= 3.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   for (double x = 1.0; x <= 1.5; x += _resolution)
//     for (double y = -0.2; y <= 0.8; y += _resolution)
//       for (double z = 0.0; z <= 3.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   _map_ok = true;
// }

// void CustomMapGenerate3(void)
// {
//   pcl::PointXYZ pt;

//   for (double x = -2.5; x <= -2.3; x += _resolution)
//     for (double y = -1.5; y <= 0.5; y += _resolution)
//       for (double z = 0.0; z <= 3.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   for (double x = -1.1; x <= -1; x += _resolution)
//     for (double y = -1.5; y <= 0.2; y += _resolution)
//       for (double z = 0.0; z <= 2.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   for (double x = -0.5; x <= 0.5; x += _resolution)
//     for (double y = -0.5; y <= 0.5; y += _resolution)
//       for (double z = 0.0; z <= 2.0; z += _resolution)
//       {
//         pt.x = x - 1.5;
//         pt.y = y + 1.3;
//         pt.z = z;
//         cloudMap.points.push_back(pt);

//         pt.x = x + 1;
//         pt.y = y + 1.5;
//         pt.z = z;
//         cloudMap.points.push_back(pt);

//         pt.x = x + 3;
//         pt.y = y + 1;
//         pt.z = z;
//         cloudMap.points.push_back(pt);

//         pt.x = x + 2.5;
//         pt.y = y - 1.5;
//         pt.z = z;
//         cloudMap.points.push_back(pt);

//         pt.x = x + 3;
//         pt.y = y - 1;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   for (double x = -0.1; x <= 0.1; x += _resolution)
//     for (double y = -0.1; y <= 0.1; y += _resolution)
//       for (double z = 0.0; z <= 2.0; z += _resolution)
//       {
//         pt.x = x - 0.5;
//         pt.y = y - 1;
//         pt.z = z;
//         cloudMap.points.push_back(pt);

//         pt.x = x + 0.5;
//         pt.y = y - 1;
//         pt.z = z;
//         cloudMap.points.push_back(pt);

//         pt.x = x + 2.5;
//         pt.y = y + 0.5;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   for (double r = 0.5; r <= 0.8; r += _resolution)
//     for (double theta = 0; theta <= 2 * 3.15; theta += 0.05)
//       for (double x = -0.2; x <= 0; x += _resolution)
//       {
//         pt.x = x;
//         pt.y = cos(theta) * r - 0.3;
//         pt.z = sin(theta) * r + 1.0;
//         cloudMap.points.push_back(pt);
//       }

//   for (double r = 0; r <= 0.5; r += _resolution)
//     for (double theta1 = 0; theta1 <= 2 * 3.15; theta1 += 0.05)
//       for (double theta2 = -3.15 / 2; theta2 <= 3.15 / 2; theta2 += 0.05)
//       {
//         pt.x = r * cos(theta2) * sin(theta1) + 1.5;
//         pt.y = r * cos(theta2) * cos(theta1) + 0.1;
//         pt.z = r * sin(theta2) + 0.9;
//         cloudMap.points.push_back(pt);
//       }

//   // for (double x = -5; x <= 5; x+= _resolution)
//   //   for (double y = -5; y <= 5; y+= _resolution)
//   //     for (double z = -0.2; z <= 0; z+= _resolution)
//   //     {
//   //       pt.x = x;
//   //       pt.y = y;
//   //       pt.z = z;
//   //       cloudMap.points.push_back(pt);
//   //     }

//   _map_ok = true;
// }

// void CustomMapGenerate4(void)
// {
//   pcl::PointXYZ pt;

//   // generate polar obs
//   for (double x = -0.25; x <= 0.25; x += _resolution)
//     for (double y = -5.0; y <= -0.4; y += _resolution)
//       for (double z = 0.0; z <= 5.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   for (double x = -0.25; x <= 0.25; x += _resolution)
//     for (double y = 0.4; y <= 5.0; y += _resolution)
//       for (double z = 0.0; z <= 5.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   for (double x = -0.25; x <= 0.25; x += _resolution)
//     for (double y = -1.0; y <= 1.0; y += _resolution)
//       for (double z = 0.0; z <= 0.7; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   for (double x = -0.25; x <= 0.25; x += _resolution)
//     for (double y = -1.0; y <= 1.0; y += _resolution)
//       for (double z = 1.5; z <= 5.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   _map_ok = true;
// }

// void CustomMapGenerate5(void)
// {
//   pcl::PointXYZ pt;

//   // generate polar obs
//   for (double x = -13; x <= -12.95; x += _resolution)
//     for (double y = 0; y <= 0.025; y += _resolution)
//       for (double z = 0.0; z <= 5.0; z += _resolution)
//       {
//         pt.x = x;
//         pt.y = y;
//         pt.z = z;
//         cloudMap.points.push_back(pt);
//       }

//   _map_ok = true;
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "map_gen");
//   ros::NodeHandle n("~");

//   _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("random_forest", 1);
//   _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

//   n.param("init_state_x", _init_x, 0.0);
//   n.param("init_state_y", _init_y, 0.0);

//   n.param("map/x_size", _x_size, 50.0);
//   n.param("map/y_size", _y_size, 50.0);
//   n.param("map/z_size", _z_size, 5.0);
//   n.param("map/obs_num", _obs_num, 30);
//   n.param("map/resolution", _resolution, 0.1);
//   n.param("map/circle_num", circle_num_, 30);
//   n.param("map/file_dir", _map_file_dir, string(""));

//   n.param("ObstacleShape/lower_rad", _w_l, 0.3);
//   n.param("ObstacleShape/upper_rad", _w_h, 0.8);
//   n.param("ObstacleShape/lower_hei", _h_l, 3.0);
//   n.param("ObstacleShape/upper_hei", _h_h, 7.0);

//   n.param("ObstacleShape/radius_l", radius_l_, 7.0);
//   n.param("ObstacleShape/radius_h", radius_h_, 7.0);
//   n.param("ObstacleShape/z_l", z_l_, 7.0);
//   n.param("ObstacleShape/z_h", z_h_, 7.0);
//   n.param("ObstacleShape/theta", theta_, 7.0);

//   n.param("sensing/radius", _sensing_range, 10.0);
//   n.param("sensing/radius", _pub_rate, 10.0);

//   // 714644379 : Return = -1008, A rounding error
//   unsigned int seed = rd();
//   // unsigned int seed = 714644379;
//   cout << "seed=" << seed << endl;
//   eng.seed(seed);

//   _x_l = -_x_size / 2.0;
//   _x_h = +_x_size / 2.0;

//   _y_l = -_y_size / 2.0;
//   _y_h = +_y_size / 2.0;

//   _obs_num = min(_obs_num, (int)_x_size * 10);
//   _z_limit = _z_size;

//   // RandomMapGenerate();
//   // PseudoRandomMapGenerate();
//   CustomMapGenerate5();

//   // bool flag_record_pcd_file = true;
//   // bool flag_use_ROI = false;
//   // int num;
//   // string _map_file_name_id = _map_file_dir + string("number.txt");
//   // fp_.open(_map_file_name_id);
//   // fp_.seekg(0);
//   // fp_ >> num;
//   // _map_file_name = _map_file_dir + to_string(num) + string(".pcd");
//   // if ( !flag_record_pcd_file && pcl::io::loadPCDFile<pcl::PointXYZ>(_map_file_name.c_str(), cloudMap_buf) == 0 )
//   // {
//   //   cout << "cloudMap.size()=" << cloudMap_buf.size() << endl;

//   //   if ( flag_use_ROI )
//   //   {
//   //     for ( size_t i=0; i<cloudMap_buf.points.size(); ++i )
//   //     {
//   //       if ( cloudMap_buf.points[i].x < 0 && cloudMap_buf.points[i].x > -4 &&
//   //           cloudMap_buf.points[i].y < 2 && cloudMap_buf.points[i].y > -2)
//   //       {
//   //         cloudMap.points.push_back( cloudMap_buf.points[i] );
//   //       }
//   //     }
//   //   }
//   //   else
//   //   {
//   //     cloudMap = cloudMap_buf;
//   //   }

//   //   ROS_WARN("read PCD file");
//   // }
//   // else
//   // {
//   //   RandomMapGenerate();
//   //   if( !_map_file_name.empty() )
//   //   {
//   //     num ++;
//   //     _map_file_name = _map_file_dir + to_string(num) + string(".pcd");
//   //     pcl::io::savePCDFile(_map_file_name.c_str(), cloudMap);

//   //     fp_.seekp(0);
//   //     fp_ << num << endl;
//   //   }
//   // }

//   // pcl::toROSMsg(cloudMap, globalMap_pcd);
//   // globalMap_pcd.header.frame_id = "map";
//   // _all_map_pub.publish(globalMap_pcd);

//   srand(0);
//   while (ros::ok())
//   {
//     // RandomMapGenerate();
//     // PseudoRandomMapGenerate();
//     // pcl::io::savePCDFile(_map_file_name.c_str(), cloudMap);

//     pcl::toROSMsg(cloudMap, globalMap_pcd);
//     globalMap_pcd.header.frame_id = "world";
//     _all_map_pub.publish(globalMap_pcd);

//     ros::spinOnce();
//     ros::Duration(1.0).sleep();
//   }

//   fp_.close();
// }
