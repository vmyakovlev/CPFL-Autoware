cmake_minimum_required(VERSION 2.8.3)
project(audio_comm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES runtime_manager
  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)
