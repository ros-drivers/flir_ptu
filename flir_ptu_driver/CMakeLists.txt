cmake_minimum_required(VERSION 2.8.3)
project(flir_ptu_driver)

find_package(catkin REQUIRED COMPONENTS actionlib diagnostic_updater roscpp roslaunch roslint rospy serial sensor_msgs tf)
find_package(Boost REQUIRED)

catkin_package(
   LIBRARIES flir_ptu_driver
   CATKIN_DEPENDS sensor_msgs serial
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(flir_ptu_driver src/driver.cpp src/tcp_client.cpp)
target_link_libraries(flir_ptu_driver ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(flir_ptu_node src/node.cpp)
target_link_libraries(flir_ptu_node ${catkin_LIBRARIES} flir_ptu_driver)
add_dependencies(flir_ptu_node flir_ptu_msgs_generate_messages_cpp)
set_target_properties(flir_ptu_node
                      PROPERTIES OUTPUT_NAME ptu_node PREFIX "")

install(TARGETS flir_ptu_driver flir_ptu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/cmd_angles
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Tests
roslint_cpp()
roslint_add_test()
roslaunch_add_file_check(launch/ptu.launch
  DEPENDENCIES flir_ptu_example_urdf flir_ptu_node)
