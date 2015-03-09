cmake_minimum_required(VERSION 2.8.3)
project(flir_ptu_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package()

set(FLIR_PTU_DEVEL_SHARE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
file(MAKE_DIRECTORY ${FLIR_PTU_DEVEL_SHARE}/urdf)

xacro_add_xacro_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/example_d46.urdf.xacro
  ${FLIR_PTU_DEVEL_SHARE}/urdf/example_d46.urdf)
xacro_add_xacro_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/example_d48.urdf.xacro
  ${FLIR_PTU_DEVEL_SHARE}/urdf/example_d48.urdf)

add_custom_target(flir_ptu_example_urdf ALL
  DEPENDS ${FLIR_PTU_DEVEL_SHARE}/urdf/example_d46.urdf
          ${FLIR_PTU_DEVEL_SHARE}/urdf/example_d48.urdf)

install(FILES 
    urdf/d46.urdf.xacro
    urdf/d48.urdf.xacro
    urdf/example_d46.urdf.xacro
    urdf/example_d48.urdf.xacro
    ${FLIR_PTU_DEVEL_SHARE}/urdf/example_d46.urdf
    ${FLIR_PTU_DEVEL_SHARE}/urdf/example_d48.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
