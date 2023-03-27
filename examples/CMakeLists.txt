cmake_minimum_required(VERSION 3.0.2)
project(examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

