cmake_minimum_required(VERSION 2.8.3)
project(mysim)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy 
  gazebo_ros
  std_msgs
  geometry_msgs 
)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS 
    roscpp
    rospy 
    gazebo_ros
    std_msgs
    geometry_msgs 
)
