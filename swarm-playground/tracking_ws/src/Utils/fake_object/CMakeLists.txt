cmake_minimum_required(VERSION 2.8.3)
project(fake_object)

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
)

catkin_package(
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(fake_object
  src/fake_object.cpp 
  )
target_link_libraries(fake_object 
  ${catkin_LIBRARIES}
  )


