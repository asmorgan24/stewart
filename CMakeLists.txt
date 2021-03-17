cmake_minimum_required(VERSION 2.8.3)
project(stewart)

SET(Eigen3_INCLUDE_DIRS "/usr/local/include/eigen3")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  gazebo_msgs
  roslib
  std_msgs
)


find_package(catkin REQUIRED COMPONENTS
   eigen_conversions
)

catkin_package(DEPENDS Eigen3)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
