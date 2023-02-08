import sys
import numpy as np
import os
import math

map_size = [20, 20, 3]
drone_diameter = 0.2
maximum_velocity = 1.5
average_obstacle_spacing = 2.5
initial_drone_position = [-12, 0, 1]

goal_type = "predefined_goals"  # "manual_goals" or "predefined_goals"
predefined_goals = [[0, 8, 1], [8, 0, 1], [0, -8, 1], [-8, 0, 1], [0, 0, 1]]


#          0 _____            |_____|___________|_____|
#            X_____\           |_____| / _____ \ |_____|
#    .-^-.  ||_| |_||  .-^-.   |_____|/ /\___/\ \|_____|
#   /_\_/_\_|  |_|  |_/_\_/_\ /|====|__/_/___\_\__|====|\
#   ||(_)| __\_____/__ |(_)|| ||====|  _/_\_/_\_  |====||   ______________
#   \/| | |::|\```/|::| | |\/ \|====| | \ ... / | |====|/  `--------------'
#   /`---_|::|-+-+-|::|_---'\       |__\ `---' /__|     _.  .--./|  |\.--.  ._
#  / /  \ |::|-|-|-|::| /  \ \       |==\_____/==|     //|  |--||----||--|  |\\
# /_/   /|`--'-+-+-`--'|\   \_\      |===|===|===|    ||__\_|  ||____||  |_/__||
# | \  / |===/_\ /_\===| \  / |      |===|+-+|===|    ||_-- |__|||==|||__| --_||
# |  \/  /---/-/-\-\  o\  \/  |      >|||<   >|||<    ||_() |___||--||___| ()_||
# | ||| | O / /   \ \   | ||| |      |---|   |---|    || --_|   ||__||   |_-- ||
# | ||| ||-------------|o|||| |      || ||   || ||    ||||  |---||__||---|  ||||
# | ||| ||----\ | /----|o|||| |      || ||   || ||     \|| /|___||__||___|\_||/
# | _|| ||-----|||-----|o|||_ |      >= =<   >= =<     |||_| \.||||||||./ |_|||
# \/|\/  |     |||     |o|\/|\/      |===|   |===|     \ _ /   \--==--/   \ _ /
# \_o/   |----|||||----|-' \o_/      >---/   \---<      <_>  /----------\  <_>
#        |##  |   |  ##|             ||#|     |#||      ||| _\__ |  | __/_ |||
#        |----|   |----|             ||-|\   /|-||      ||| \  |\|  |/|  / |||
#        ||__ |   | __||             ||+||   ||+||      ||| |  |_|__|_|  | |||
#       [|'  `|] [|'  `|]            ||-|/   \|-||      ||| [--+ \  / +--] |||
#       [|`--'|] [|`--'|]            ||_|\   /|_||      ||| |--+-/  \-+--| |||
#       /|__| |\ /| |__|\         ___|/-\/   \/-\|___   ||| |  ||    ||  | |||
#       ||  | || || |  ||        /________\ /________\  |=| |___|    |___| |=|
#       ||__|_|| ||_|__||                               / \ |---|    |---| / \
#       ||    || ||    ||                               |=| | | |    | | | |=|
#       \|----|/ \|----|/                               \ / |___|    |___| \ /
#       /______\ /______\                     ^          = (| | ||  || | |) =
#       |__||__| |__||__|                   (/U\)           |--_||  ||_--|
#                                           \___/     /    _|_#__|  |__#_|_
#      ____          ____               --  _|||_  --/    /______\  /______\
#     |oooo|        |oooo|             //\\/ ___ \/,/\   |________||________|
#     |oooo| .----. |oooo|             \\////_^_\\\/'/
#     |Oooo|/\_||_/\|oooO|              -///-/_\-\/\-
#     `----' / __ \ `----'             |||/_______\|||
#     ,/ |#|/\/__\/\|#| \,           ,/|||\  |O|  /|||\,
#    /  \|#|| |/\| ||#|/  \         /  |_\\/-___-\//_|  \
#   / \_/|_|| |/\| ||_|\_/ \       / ,/.--/-------\--.\, \
#  |_\/    o\=----=/o    \/_|     /\/  | / ||`|'|| \ |  \/\
#  <_>      |=\__/=|      <_>    / /    ||=|`---'|=||    \ \
#  <_>      |------|      <_>   |\/     |--|     |--|     \/|
#  | |   ___|======|___   | |  /()\     \__/     \__/     /()\
# //\\  / |O|======|O| \  //\\ \__/     /  \     /  \     \__/
# |  |  | |O+------+O| |  |  | |  |     ||||     ||||     |  |
# |\/|  \_+/        \+_/  |\/| |==|     ||||     ||||     |==|
# \__/  _|||        |||_  \__/          ||||     ||||
#       | ||        || |                ||||     ||||
#      [==|]        [|==]               \__/     \__/
#      [===]        [===]               /  \     /  \
#       >_<          >_<                ||||     ||||
#      || ||        || ||               ||||     ||||
#      || ||        || ||               ||||     ||||
#      || ||        || ||             ,/|__|\   /|__|\,
#    __|\_/|__    __|\_/|__          /--|__|-\ /-|__|--\
#   /___n_n___\  /___n_n___\        |___|  |_| |_|  |___|


##########################################
# 1. autoflight.launch
##########################################

fname = "autoflight.launch"
file = open(fname, "w")
obstacle_num = (map_size[0]*map_size[1]*map_size[2]) / \
    (3.14*pow(average_obstacle_spacing/2, 2))
str = "\
<launch>\n\
    <arg name=\"map_size_x\" value=\"{map_size_x}\"/>\n\
    <arg name=\"map_size_y\" value=\"{map_size_y}\"/>\n\
    <arg name=\"map_size_z\" value=\"{map_size_z}\"/>\n\
    <arg name=\"odom_topic\" value=\"visual_slam/odom\" />\n\
    \n\
    <!-- map -->\n\
    <node pkg =\"map_generator\" name =\"random_forest\" type =\"random_forest\" output = \"screen\">\n\
        <param name=\"map/x_size\" value=\"{map_size_x_obs}\" />\n\
        <param name=\"map/y_size\" value=\"{map_size_y_obs}\" />\n\
        <param name=\"map/z_size\" value=\"{map_size_z_obs}\" />\n\
        <param name=\"map/resolution\" value=\"0.1\"/>\n\
        <param name=\"ObstacleShape/seed\" value=\"1\"/>\n\
        <param name=\"map/obs_num\" value=\"{obs_num}\"/>\n\
        <param name=\"ObstacleShape/lower_rad\" value=\"0.5\"/>\n\
        <param name=\"ObstacleShape/upper_rad\" value=\"0.7\"/>\n\
        <param name=\"ObstacleShape/lower_hei\" value=\"0.0\"/>\n\
        <param name=\"ObstacleShape/upper_hei\" value=\"3.0\"/>\n\
        <param name=\"map/circle_num\" value=\"{obs_num}\"/>\n\
        <param name=\"ObstacleShape/radius_l\" value=\"0.7\"/>\n\
        <param name=\"ObstacleShape/radius_h\" value=\"0.5\"/>\n\
        <param name=\"ObstacleShape/z_l\" value=\"0.7\"/>\n\
        <param name=\"ObstacleShape/z_h\" value=\"0.8\"/>\n\
        <param name=\"ObstacleShape/theta\" value=\"0.5\"/>\n\
        <param name=\"pub_rate\" value=\"1.0\"/>\n\
        <param name=\"min_distance\" value=\"0.8\"/>\n\
    </node>\n\
    \n\
    <include file=\"$(find ego_planner)/launch/include/run_in_sim.xml\">\n\
        <arg name=\"drone_id\" value=\"0\"/>\n\
        <arg name=\"init_x\" value=\"{init_x}\"/>\n\
        <arg name=\"init_y\" value=\"{init_y}\"/>\n\
        <arg name=\"init_z\" value=\"{init_z}\"/>\n\
        <arg name=\"map_size_x\" value=\"$(arg map_size_x)\"/>\n\
        <arg name=\"map_size_y\" value=\"$(arg map_size_y)\"/>\n\
        <arg name=\"map_size_z\" value=\"$(arg map_size_z)\"/>\n\
        <arg name=\"odom_topic\" value=\"$(arg odom_topic)\"/>\n\
        <arg name=\"point_num\" value=\"{num}\"/>\n".\
    format(num=len(predefined_goals),
           map_size_x=map_size[0]+4, map_size_y=map_size[1]+4, map_size_z=map_size[2]+4,
           map_size_x_obs=map_size[0], map_size_y_obs=map_size[1], map_size_z_obs=map_size[2],
           obs_num=obstacle_num/2,
           init_x=initial_drone_position[0], init_y=initial_drone_position[1], init_z=initial_drone_position[2])
file.write(str)

for i in range(len(predefined_goals)):
    str = "\
        <arg name=\"target{id}_x\" value=\"{x}\"/>\n\
        <arg name=\"target{id}_y\" value=\"{y}\"/>\n\
        <arg name=\"target{id}_z\" value=\"{z}\"/>\n".format(
        id=i, x=predefined_goals[i][0], y=predefined_goals[i][1], z=predefined_goals[i][2])
    file.write(str)

str = "\
    </include>\n\
</launch>\n\
    "

file.write(str)
file.close()

##########################################
# 2. run_in_sim.xml
##########################################

fname = "include/run_in_sim.xml"
file = open(fname, "w")
str = "\
<launch>\n\
    <!-- size of map, change the size inflate x, y, z according to your application -->\n\
    <arg name=\"drone_id\" default=\"0\"/>\n\
    <arg name=\"map_size_x\"/>\n\
    <arg name=\"map_size_y\"/>\n\
    <arg name=\"map_size_z\"/>\n\
    <arg name=\"init_x\"/>\n\
    <arg name=\"init_y\"/>\n\
    <arg name=\"init_z\"/>\n\
    <arg name=\"flight_type\" default=\"{flight_type}\"/>\n\
    <arg name=\"point_num\" default=\"1\"/>\n".format(flight_type=1 if goal_type == "manual_goals" else 2)
file.write(str)

for i in range(len(predefined_goals)):
    str = "\
    <arg name=\"target{id}_x\" default=\"0.0\"/>\n\
    <arg name=\"target{id}_y\" default=\"0.0\"/>\n\
    <arg name=\"target{id}_z\" default=\"0.0\"/>\n".format(id=i)
    file.write(str)

str = "\
    <!-- topic of your odometry such as VIO or LIO -->\n\
    <arg name=\"odom_topic\"/>\n\
    <!-- main algorithm params -->\n\
    <include file=\"$(find ego_planner)/launch/include/advanced_param.xml\">\n\
        <arg name=\"drone_id\" value=\"$(arg drone_id)\"/>\n\
        <arg name=\"map_size_x_\" value=\"$(arg map_size_x)\"/>\n\
        <arg name=\"map_size_y_\" value=\"$(arg map_size_y)\"/>\n\
        <arg name=\"map_size_z_\" value=\"$(arg map_size_z)\"/>\n\
        <arg name=\"odometry_topic\" value=\"$(arg odom_topic)\"/>\n\
        <!-- camera pose: transform of camera frame in the world frame -->\n\
        <!-- depth topic: depth image, 640x480 by default -->\n\
        <!-- don't set cloud_topic if you already set these ones! -->\n\
        <arg name=\"camera_pose_topic\" value=\"pcl_render_node/camera_pose\"/>\n\
        <arg name=\"depth_topic\" value=\"pcl_render_node/depth\"/>\n\
        <!-- topic of point cloud measurement, such as from LIDAR  -->\n\
        <!-- don't set camera pose and depth, if you already set this one! -->\n\
        <arg name=\"cloud_topic\" value=\"pcl_render_node/cloud\"/>\n\
        <!-- intrinsic params of the depth camera -->\n\
        <arg name=\"cx\" value=\"321.04638671875\"/>\n\
        <arg name=\"cy\" value=\"243.44969177246094\"/>\n\
        <arg name=\"fx\" value=\"387.229248046875\"/>\n\
        <arg name=\"fy\" value=\"387.229248046875\"/>\n\
        <!-- maximum velocity, acceleration and jerk the drone will reach -->\n\
        <arg name=\"max_vel\" value=\"{max_vel}\" />\n\
        <arg name=\"max_acc\" value=\"6.0\" />\n\
        <arg name=\"max_jer\" value=\"20.0\" />\n\
        <!--always set to 1.5 times grater than sensing horizen-->\n\
        <arg name=\"planning_horizon\" value=\"7.5\" />\n\
        <arg name=\"use_multitopology_trajs\" value=\"false\" />\n\
        <!-- 1: use 2D Nav Goal to select goal  -->\n\
        <!-- 2: use global waypoints below  -->\n\
        <arg name=\"flight_type\" value=\"$(arg flight_type)\" />\n\
        <!-- global waypoints -->\n\
        <!-- It generates a piecewise min-snap traj passing all waypoints -->\n\
        <arg name=\"point_num\" value=\"$(arg point_num)\" />\n".format(max_vel=maximum_velocity)
file.write(str)

for i in range(len(predefined_goals)):
    str = "\
        <arg name=\"point{id}_x\" value=\"$(arg target{id}_x)\" />\n\
        <arg name=\"point{id}_y\" value=\"$(arg target{id}_y)\" />\n\
        <arg name=\"point{id}_z\" value=\"$(arg target{id}_z)\" />\n".format(id=i)
    file.write(str)

str = "\
    </include>\n\
    <!-- trajectory server -->\n\
    <node pkg=\"ego_planner\" name=\"drone_$(arg drone_id)_traj_server\" type=\"traj_server\" output=\"screen\">\n\
        <remap from=\"position_cmd\" to=\"drone_$(arg drone_id)_planning/pos_cmd\"/>\n\
        <remap from=\"~planning/trajectory\" to=\"drone_$(arg drone_id)_planning/trajectory\"/>\n\
        <param name=\"traj_server/time_forward\" value=\"1.0\" type=\"double\"/>\n\
    </node>\n\
    <!-- use simulator -->\n\
    <include file=\"$(find ego_planner)/launch/include/simulator.xml\">\n\
        <arg name=\"drone_id\" value=\"$(arg drone_id)\"/>\n\
        <arg name=\"map_size_x_\" value=\"$(arg map_size_x)\"/>\n\
        <arg name=\"map_size_y_\" value=\"$(arg map_size_y)\"/>\n\
        <arg name=\"map_size_z_\" value=\"$(arg map_size_z)\"/>\n\
        <arg name=\"init_x_\" value=\"$(arg init_x)\"/>\n\
        <arg name=\"init_y_\" value=\"$(arg init_y)\"/>\n\
        <arg name=\"init_z_\" value=\"$(arg init_z)\"/>\n\
        <arg name=\"odometry_topic\" value=\"$(arg odom_topic)\" />\n\
    </include>\n\
    <include file=\"$(find manual_take_over)/launch/take_over_drone.launch\">\n\
        <arg name=\"drone_id\" value=\"$(arg drone_id)\"/>\n\
        <arg name=\"cmd_topic\" value=\"drone_$(arg drone_id)_planning/pos_cmd\"/>\n\
    </include>\n\
</launch>\n\
    "

file.write(str)
file.close()

##########################################
# advanced_param.xml
##########################################

fname = "include/advanced_param.xml"
file = open(fname, "w")
str = "\
<launch>\n\
    <arg name=\"map_size_x_\"/>\n\
    <arg name=\"map_size_y_\"/>\n\
    <arg name=\"map_size_z_\"/>\n\
    <arg name=\"odometry_topic\"/>\n\
    <arg name=\"camera_pose_topic\"/>\n\
    <arg name=\"depth_topic\"/>\n\
    <arg name=\"cloud_topic\"/>\n\
    <arg name=\"cx\"/>\n\
    <arg name=\"cy\"/>\n\
    <arg name=\"fx\"/>\n\
    <arg name=\"fy\"/>\n\
    <arg name=\"max_vel\"/>\n\
    <arg name=\"max_acc\"/>\n\
    <arg name=\"max_jer\"/>\n\
    <arg name=\"planning_horizon\"/>\n\
    <arg name=\"point_num\"/>\n"
file.write(str)

for i in range(len(predefined_goals)):
    str = "\
    <arg name=\"point{id}_x\"/>\n\
    <arg name=\"point{id}_y\"/>\n\
    <arg name=\"point{id}_z\"/>\n".format(id=i)
    file.write(str)

str = "\
    <arg name=\"flight_type\"/>\n\
    <arg name=\"use_multitopology_trajs\"/>\n\
    <arg name=\"drone_id\"/>\n\
    <!-- main node -->\n\
    <!-- <node pkg=\"ego_planner\" name=\"ego_planner_node\" type=\"ego_planner_node\" output=\"screen\" launch-prefix=\"valgrind\"> -->\n\
    <node pkg=\"ego_planner\" name=\"drone_$(arg drone_id)_ego_planner_node\" type=\"ego_planner_node\" output=\"screen\">\n\
        <remap from=\"~odom_world\" to=\"/drone_$(arg drone_id)_$(arg odometry_topic)\"/>\n\
        <remap from=\"~mandatory_stop\" to=\"/mandatory_stop_to_planner\"/>\n\
        <remap from=\"~planning/trajectory\" to = \"/drone_$(arg drone_id)_planning/trajectory\"/>\n\
        <remap from=\"~planning/data_display\" to = \"/drone_$(arg drone_id)_planning/data_display\"/>\n\
        <remap from=\"~planning/broadcast_traj_send\" to = \"/broadcast_traj_from_planner\"/>\n\
        <remap from=\"~planning/broadcast_traj_recv\" to = \"/broadcast_traj_to_planner\"/>\n\
        <remap from=\"~planning/heartbeat\" to = \"/drone_$(arg drone_id)_traj_server/heartbeat\"/>\n\
        <remap from=\"/goal\" to = \"/goal_with_id\"/>\n\
        <remap from=\"~grid_map/odom\" to=\"/drone_$(arg drone_id)_$(arg odometry_topic)\"/>\n\
        <remap from=\"~grid_map/cloud\" to=\"/drone_$(arg drone_id)_$(arg cloud_topic)\"/>\n\
        <remap from=\"~grid_map/pose\"   to = \"/drone_$(arg drone_id)_$(arg camera_pose_topic)\"/>\n\
        <remap from=\"~grid_map/depth\" to = \"/drone_$(arg drone_id)_$(arg depth_topic)\"/>\n\
        <!-- planning fsm -->\n\
        <param name=\"fsm/flight_type\" value=\"$(arg flight_type)\" type=\"int\"/>\n\
        <param name=\"fsm/thresh_replan_time\" value=\"1.0\" type=\"double\"/>\n\
        <param name=\"fsm/planning_horizon\" value=\"$(arg planning_horizon)\" type=\"double\"/>\n\
        <!--always set to 1.5 times grater than sensing horizen-->\n\
        <param name=\"fsm/emergency_time\" value=\"1.0\" type=\"double\"/>\n\
        <param name=\"fsm/realworld_experiment\" value=\"false\"/>\n\
        <param name=\"fsm/fail_safe\" value=\"true\"/>\n\
        <param name=\"fsm/waypoint_num\" value=\"$(arg point_num)\" type=\"int\"/>\n"
file.write(str)

for i in range(len(predefined_goals)):
    str = "\
        <param name=\"fsm/waypoint{id}_x\" value=\"$(arg point{id}_x)\" type=\"double\"/>\n\
        <param name=\"fsm/waypoint{id}_y\" value=\"$(arg point{id}_y)\" type=\"double\"/>\n\
        <param name=\"fsm/waypoint{id}_z\" value=\"$(arg point{id}_z)\" type=\"double\"/>\n".format(id=i)
    file.write(str)

inflation = drone_diameter / 2
resolution = (inflation + 0.01) / 3 if (inflation + 0.01) / 3 > 0.1 else 0.1
swarm_clearance = drone_diameter / 2 * 1.5
str = "\
        <param name=\"grid_map/resolution\" value=\"{reso}\" />\n\
        <param name=\"grid_map/map_size_x\" value=\"$(arg map_size_x_)\" />\n\
        <param name=\"grid_map/map_size_y\" value=\"$(arg map_size_y_)\" />\n\
        <param name=\"grid_map/map_size_z\" value=\"$(arg map_size_z_)\" />\n\
        <param name=\"grid_map/local_update_range_x\" value=\"5.5\" />\n\
        <param name=\"grid_map/local_update_range_y\" value=\"5.5\" />\n\
        <param name=\"grid_map/local_update_range_z\" value=\"2.0\" /> <!-- different meaning between grid_map.cpp and grid_map_bigmap.cpp -->\n\
        <param name=\"grid_map/obstacles_inflation\" value=\"{infl}\" />\n\
        <param name=\"grid_map/local_map_margin\" value=\"10\"/>\n\
        <param name=\"grid_map/enable_virtual_wall\" value=\"true\"/>\n\
        <param name=\"grid_map/virtual_ceil\" value=\"3.0\"/>\n\
        <param name=\"grid_map/virtual_ground\" value=\"-0.1\"/>\n\
        <param name=\"grid_map/ground_height\" value=\"-0.01\"/>\n\
        <!-- camera parameter -->\n\
        <param name=\"grid_map/cx\" value=\"$(arg cx)\"/>\n\
        <param name=\"grid_map/cy\" value=\"$(arg cy)\"/>\n\
        <param name=\"grid_map/fx\" value=\"$(arg fx)\"/>\n\
        <param name=\"grid_map/fy\" value=\"$(arg fy)\"/>\n\
        <!-- depth filter -->\n\
        <param name=\"grid_map/use_depth_filter\" value=\"true\"/>\n\
        <param name=\"grid_map/depth_filter_tolerance\" value=\"0.15\"/>\n\
        <param name=\"grid_map/depth_filter_maxdist\" value=\"5.0\"/>\n\
        <param name=\"grid_map/depth_filter_mindist\" value=\"0.2\"/>\n\
        <param name=\"grid_map/depth_filter_margin\" value=\"2\"/>\n\
        <param name=\"grid_map/k_depth_scaling_factor\" value=\"1000.0\"/>\n\
        <param name=\"grid_map/skip_pixel\" value=\"2\"/>\n\
        <!-- local fusion -->\n\
        <param name=\"grid_map/p_hit\" value=\"0.65\"/>\n\
        <param name=\"grid_map/p_miss\" value=\"0.35\"/>\n\
        <param name=\"grid_map/p_min\" value=\"0.12\"/>\n\
        <param name=\"grid_map/p_max\" value=\"0.90\"/>\n\
        <param name=\"grid_map/p_occ\" value=\"0.80\"/>\n\
        <param name=\"grid_map/fading_time\" value=\"1000.0\"/>\n\
        <param name=\"grid_map/min_ray_length\" value=\"0.1\"/>\n\
        <param name=\"grid_map/max_ray_length\" value=\"4.5\"/>\n\
        <param name=\"grid_map/visualization_truncate_height\" value=\"1.9\"/>\n\
        <param name=\"grid_map/show_occ_time\" value=\"false\"/>\n\
        <param name=\"grid_map/pose_type\" value=\"1\"/>\n\
        <param name=\"grid_map/frame_id\" value=\"world\"/>\n\
        <!-- planner manager -->\n\
        <param name=\"manager/max_vel\" value=\"$(arg max_vel)\" type=\"double\"/>\n\
        <param name=\"manager/max_acc\" value=\"$(arg max_acc)\" type=\"double\"/>\n\
        <param name=\"manager/polyTraj_piece_length\" value=\"1.5\" type=\"double\"/>\n\
        <param name=\"manager/feasibility_tolerance\" value=\"0.05\" type=\"double\"/>\n\
        <param name=\"manager/planning_horizon\" value=\"$(arg planning_horizon)\" type=\"double\"/>\n\
        <param name=\"manager/use_multitopology_trajs\" value=\"$(arg use_multitopology_trajs)\" type=\"bool\"/>\n\
        <param name=\"manager/drone_id\" value=\"$(arg drone_id)\"/>\n\
        <!-- trajectory optimization -->\n\
        <param name=\"optimization/constraint_points_perPiece\" value=\"5\" type=\"int\"/>\n\
        <param name=\"optimization/weight_obstacle\" value=\"10000.0\" type=\"double\"/>\n\
        <param name=\"optimization/weight_obstacle_soft\" value=\"5000.0\" type=\"double\"/>\n\
        <param name=\"optimization/weight_swarm\" value=\"10000.0\" type=\"double\"/>\n\
        <param name=\"optimization/weight_feasibility\" value=\"10000.0\" type=\"double\"/>\n\
        <param name=\"optimization/weight_sqrvariance\" value=\"10000.0\" type=\"double\"/>\n\
        <param name=\"optimization/weight_time\" value=\"10.0\" type=\"double\"/>\n\
        <param name=\"optimization/obstacle_clearance\" value=\"0.1\" type=\"double\"/>\n\
        <param name=\"optimization/obstacle_clearance_soft\" value=\"0.5\" type=\"double\"/>\n\
        <param name=\"optimization/swarm_clearance\" value=\"{sc}\" type=\"double\"/> <!-- heterogeneous support: my required clearance -->\n\
        <param name=\"optimization/max_vel\" value=\"$(arg max_vel)\" type=\"double\"/>\n\
        <param name=\"optimization/max_acc\" value=\"$(arg max_acc)\" type=\"double\"/>\n\
        <param name=\"optimization/max_jer\" value=\"$(arg max_jer)\" type=\"double\"/>\n\
        <param name=\"optimization/record_opt\" value=\"true\"/>\n\
    </node>\n\
</launch>\n".format(reso=resolution, infl=inflation, sc=swarm_clearance)
file.write(str)
file.close()
