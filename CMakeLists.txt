cmake_minimum_required(VERSION 3.0.2)
project(ars_optitrack_transf)

find_package(catkin REQUIRED COMPONENTS
  rospy
  # - Messages
  std_msgs
  geometry_msgs
)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)
