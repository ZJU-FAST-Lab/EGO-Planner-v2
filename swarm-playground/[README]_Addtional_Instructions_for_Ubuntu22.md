# Getting Started with "Conda ROS"

First please install `mamba`, which is a faster version of `conda` and defaults to the `conda-forge` channel. Then you can run the following to set up the environment.

```sh
mamba create -n ego-v2 python=3.9 -y
mamba activate ego-v2

mamba install ros-noetic-desktop-full=1.5.0 -c robostack-staging -y
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools -y
mamba install armadillo=12.6.5 -y

# reactivate the env to prevent permission issues
mamba deactivate
mamba activate ego-v2
```

With this environment activated you can run ROS commands as normal. For further instructions please refer to the [tutorial pdf](./[README]_Brief_Documentation_for_Swarm_Playground.pdf).
