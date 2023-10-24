
cmake_minimum_required(VERSION 3.5)
project(KalmanFilter)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)


find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)


# Add include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${eigen3_INCLUDE_DIRS}
)

# Add executable
add_executable(kalman_filter_node src/KalmanFilter.cpp)

# Link libraries
ament_target_dependencies(kalman_filter_node rclcpp geometry_msgs eigen3)

# Install executable
install(TARGETS kalman_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include
  DESTINATION include
)

# Install package.xml
ament_package()