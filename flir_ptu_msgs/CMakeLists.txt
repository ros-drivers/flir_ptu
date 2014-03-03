cmake_minimum_required(VERSION 2.8.3)
project(flir_ptu_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs sensor_msgs std_msgs)

add_action_files(
  FILES
  PtuGoto.action
  PtuReset.action
  PtuSetVel.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  sensor_msgs
  std_msgs
)

catkin_package()
