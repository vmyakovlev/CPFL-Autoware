cmake_minimum_required(VERSION 2.8.3)
project(points_localizer)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PCL REQUIRED)

IF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
SET(FAST_PCL_PACKAGES filters registration)
ENDIF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package( OpenMP )
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  autoware_msgs
  pcl_conversions
  velodyne_pointcloud
  ${FAST_PCL_PACKAGES}
#  ndt_tku
)
elseif("${ROS_VERSION}" MATCHES "(kinetic)")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  autoware_msgs
  pcl_conversions
  velodyne_pointcloud
  ${FAST_PCL_PACKAGES}
)
endif()

###################################
## catkin specific configuration ##
###################################
if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS std_msgs velodyne_pointcloud autoware_msgs ${FAST_PCL_PACKAGES}
#  DEPENDS ndt_tku
#  DEPENDS system_lib
)
elseif("${ROS_VERSION}" MATCHES "(kinetic)")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS std_msgs velodyne_pointcloud autoware_msgs ${FAST_PCL_PACKAGES}
#  DEPENDS ndt_tku
#  DEPENDS system_lib
)
endif()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

IF(PCL_VERSION VERSION_LESS "1.7.2")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
ELSE(PCL_VERSION VERSION_LESS "1.7.2")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -DUSE_FAST_PCL ${CMAKE_CXX_FLAGS}")
ENDIF(PCL_VERSION VERSION_LESS "1.7.2")

add_executable(ndt_slam_pcl nodes/ndt_slam_pcl/ndt_slam_pcl_node.cpp)
#add_executable(icp_slam_pcl nodes/icp_slam_pcl/icp_slam_pcl_node.cpp)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
#add_executable(ndt_slam_tku nodes/ndt_slam_tku/ndt_slam_tku_node.cpp)
endif()

target_link_libraries(ndt_slam_pcl ${catkin_LIBRARIES})
target_link_libraries(ndt_icp_pcl ${catkin_LIBRARIES})

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
#target_link_libraries(ndt_slam_tku ndt_tku ${catkin_LIBRARIES})
endif()

add_dependencies(ndt_slam_pcl autoware_msgs_generate_messages_cpp)
