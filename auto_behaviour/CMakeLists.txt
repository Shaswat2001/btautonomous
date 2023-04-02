cmake_minimum_required(VERSION 3.0.2)
project(auto_behaviour)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  tf2
)
catkin_python_setup()

find_package(OpenCV REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${OpenCV_LIBS}
   CATKIN_DEPENDS 
   DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
