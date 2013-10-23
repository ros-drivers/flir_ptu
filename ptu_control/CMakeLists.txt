cmake_minimum_required(VERSION 2.8.3)
project(ptu_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf)


###################################

add_action_files(
  DIRECTORY action
  FILES PtuGoto.action   PtuReset.action  PtuSetVel.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  sensor_msgs # Or other packages containing msgs
)

## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS actionlib actionlib_msgs  nav_msgs sensor_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

