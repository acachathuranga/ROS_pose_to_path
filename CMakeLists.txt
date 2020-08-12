cmake_minimum_required(VERSION 2.8.3)
project(pose_to_path)

# CMAKE minimal description for ROS package

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package( )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

