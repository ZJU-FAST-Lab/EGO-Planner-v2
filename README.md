# About This Fork

This fork supports building and testing using a ROS Noetic Conda install maintained by [Robostack](https://robostack.github.io/GettingStarted.html).

```sh
mamba install ros-noetic-desktop-full=1.5.0 -c robostack-staging -y

mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools -y

mamba install ompl=1.5.2 armadillo=12.6.5 -y
```

Now you can use it on whatever distro you like (Ubuntu Jammy is tested). See commits to find out what changes have been made. The original README is kept below, enjoy!

# EGO-Planner-v2
Swarm Playground, the codebase of the paper "[Swarm of micro flying robots in the wild](https://www.science.org/doi/10.1126/scirobotics.abm5954)".

# Installation-Free Usage

Please follow the [tutorial PDF file](swarm-playground/[README]_Brief_Documentation_for_Swarm_Playground.pdf) with corresponding videos ([1](swarm-playground/main_ws/WatchMe_main.mp4), [2](swarm-playground/formation_ws/WatchMe_formation.mp4), [3](swarm-playground/tracking_ws/WatchMe_tracking.mp4), [4](swarm-playground/interlaced_flight_ws/WatchMe_interlaced_flights.mp4)) to run the code.

This work was born out of [MINCO](https://github.com/ZJU-FAST-Lab/GCOPTER).
If you find it interesting, please give both repos stars generously. Thanks.

<img src="images/cover.jpg" alt="drawing" width="400"/>
