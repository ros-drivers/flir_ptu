cmake_minimum_required(VERSION 2.8.3)
project(flir_ptu_driver)

find_package(catkin REQUIRED COMPONENTS actionlib diagnostic_updater roscpp rospy serial sensor_msgs tf)
find_package(Boost REQUIRED)

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
   LIBRARIES flir_ptu_driver
   CATKIN_DEPENDS serial
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(flir_ptu_driver src/driver.cpp)
target_link_libraries(flir_ptu_driver ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(flir_ptu_node src/node.cpp)
target_link_libraries(flir_ptu_node ${catkin_LIBRARIES} flir_ptu_driver)
add_dependencies(flir_ptu_node flir_ptu_msgs_generate_messages_cpp)
set_target_properties(flir_ptu_node
                      PROPERTIES OUTPUT_NAME ptu_node PREFIX "")
