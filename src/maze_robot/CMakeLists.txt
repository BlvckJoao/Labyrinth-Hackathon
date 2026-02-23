cmake_minimum_required(VERSION 3.8)
project(maze_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Create navigation node executable
add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(navigation_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Create perception node executable
add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install executables
install(TARGETS
  navigation_node
  perception_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
