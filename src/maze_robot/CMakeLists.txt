cmake_minimum_required(VERSION 3.8)
project(maze_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(motor_node src/motor_node.cpp)
target_include_directories(motor_node PUBLIC include)
ament_target_dependencies(motor_node
  rclcpp
  std_msgs
)

add_executable(navigation_node src/navigation_node.cpp)
target_include_directories(navigation_node PUBLIC include)
ament_target_dependencies(navigation_node
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(perception_node src/perception_node.cpp)
target_include_directories(perception_node PUBLIC include)
ament_target_dependencies(perception_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  motor_node
  navigation_node
  perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
