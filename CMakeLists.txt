cmake_minimum_required(VERSION 3.5)
project(ros2_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# --- Clock Publisher Node ---
add_executable(clock_publisher_node src/clock_publisher.cpp)
ament_target_dependencies(clock_publisher_node rclcpp rosgraph_msgs)

# --- Time Rewrite Node ---
add_executable(time_rewrite_node src/time_rewrite_node.cpp)
ament_target_dependencies(time_rewrite_node rclcpp sensor_msgs tf2_msgs)

# -- Time Rewrite Node --
add_executable(lidar_sync_node src/lidar_sync_node.cpp)
ament_target_dependencies(lidar_sync_node rclcpp sensor_msgs tf2_msgs)

# --- Install targets ---
install(TARGETS
  clock_publisher_node
  time_rewrite_node
  lidar_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
