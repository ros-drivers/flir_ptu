cmake_minimum_required(VERSION 2.8.3)
project(flir_ptu_viz)

find_package(catkin REQUIRED COMPONENTS roslaunch)
catkin_package()

roslaunch_add_file_check(launch/view_model.launch
  DEPENDENCIES flir_ptu_example_urdf robot_state_publisher)
roslaunch_add_file_check(launch/view_ptu.launch
  DEPENDENCIES rviz)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
