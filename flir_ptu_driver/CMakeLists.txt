cmake_minimum_required(VERSION 3.8)
project(flir_ptu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_library(flir_ptu_driver
  src/driver.cpp
  src/serial_transport.cpp
  src/tcp_transport.cpp
)
target_include_directories(flir_ptu_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(flir_ptu_driver PUBLIC cxx_std_17)
ament_target_dependencies(flir_ptu_driver rclcpp)

add_executable(ptu_node src/node.cpp)
target_link_libraries(ptu_node flir_ptu_driver)
ament_target_dependencies(ptu_node
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_updater
)

install(TARGETS flir_ptu_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ptu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/cmd_angles
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_format)
  ament_lint_auto_find_test_dependencies()
  ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format)
endif()

ament_package()
