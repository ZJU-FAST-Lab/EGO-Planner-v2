cmake_minimum_required(VERSION 2.8.3)
project(assign_goals)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quadrotor_msgs
  traj_utils
  uav_utils
)

catkin_package(
 CATKIN_DEPENDS traj_utils quadrotor_msgs
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(assign_goals_node
  src/assign_goals_node.cpp 
  )
target_link_libraries(assign_goals_node 
  ${catkin_LIBRARIES}
  )


