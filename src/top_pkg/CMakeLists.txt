cmake_minimum_required(VERSION 3.8)
project(top_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(OpenCV REQUIRED)

add_executable(sync_capture_node
  src/sync_capture_node.cpp
)

ament_target_dependencies(sync_capture_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  livox_ros_driver2
)

target_include_directories(sync_capture_node PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(sync_capture_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS
  sync_capture_node
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
