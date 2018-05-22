cmake_minimum_required(VERSION 2.8.3)
project(lidar_naive_L_shape_detect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  autoware_msgs
  tf
  jsk_recognition_msgs
  jsk_rviz_plugins
  OpenCV
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(SOURCE_FILES nodes/naive_L_shape/naive_L_shape_node.cpp nodes/naive_L_shape/naive_L_shape.cpp)

add_executable(naive_L_shape ${SOURCE_FILES})

target_link_libraries(naive_L_shape ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
