cmake_minimum_required(VERSION 2.8.3)
project(flir_ptu_driver)

find_package(catkin REQUIRED COMPONENTS actionlib diagnostic_updater roscpp rospy sensor_msgs tf)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES flir_ptu
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(flir_ptu_driver
  src/driver.cpp
)

## Declare a cpp executable
add_executable(flir_ptu_node src/node.cpp)
add_dependencies(flir_ptu_driver flir_ptu_msgs_generate_messages_cpp)
target_link_libraries(flir_ptu_node
  ${catkin_LIBRARIES}
  flir_ptu_driver
)
