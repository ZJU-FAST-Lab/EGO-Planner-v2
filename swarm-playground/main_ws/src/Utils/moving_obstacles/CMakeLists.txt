cmake_minimum_required(VERSION 2.8.3)
project(moving_obstacles)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  traj_opt
  traj_utils
)

catkin_package(
  DEPENDS traj_opt traj_utils
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(moving_obstacles
  src/moving_obstacles.cpp 
  )
target_link_libraries(moving_obstacles 
  ${catkin_LIBRARIES}
  )


