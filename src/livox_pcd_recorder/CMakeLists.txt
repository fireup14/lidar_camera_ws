cmake_minimum_required(VERSION 3.8)
project(livox_pcd_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_recorder_node
  src/pcd_recorder_node.cpp
)

ament_target_dependencies(pcd_recorder_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(pcd_recorder_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  pcd_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
