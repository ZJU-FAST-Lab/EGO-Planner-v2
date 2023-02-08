cmake_minimum_required(VERSION 2.8.3)
project(manual_take_over)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quadrotor_msgs
  sensor_msgs
)

catkin_package(
  DEPENDS quadrotor_msgs
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(manual_take_over
  src/manual_take_over.cpp 
  )
target_link_libraries(manual_take_over 
  ${catkin_LIBRARIES}
  )

add_executable(manual_take_over_station
  src/ground_station.cpp 
  )
target_link_libraries(manual_take_over_station 
  ${catkin_LIBRARIES}
  )


