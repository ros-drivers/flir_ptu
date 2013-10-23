cmake_minimum_required(VERSION 2.8.3)
project(ptu_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf)

add_action_files(
  DIRECTORY action
  FILES PtuGoto.action   PtuReset.action  PtuSetVel.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  sensor_msgs # Or other packages containing msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

