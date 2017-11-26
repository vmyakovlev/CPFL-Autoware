cmake_minimum_required(VERSION 2.8.3)
project(ff_waypoint_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  runtime_manager
  waypoint_follower_msgs
  map_file
  op_utility
  op_planner
  op_simu
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs runtime_manager map_file op_utility op_planner op_simu
  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include 
#  /usr/local/include/zmp/minivan
  ${catkin_INCLUDE_DIRS}
)

find_package(CUDA)
if (CUDA_FOUND)
include_directories(${CUDA_INCLUDE_DIRS})
# Create flag to make cuda enable
set(CUDA_ENABLE_FLAGS
  "-DCUDA_FOUND"
  )
endif(CUDA_FOUND)

## Declare a C++ library
#add_library(libplanner_x lib/libplanner_x.cpp)

link_directories(lib)
if(CUDA_FOUND)
cuda_add_executable(ff_waypoint_follower nodes/ff_waypoint_follower.cpp nodes/ff_waypoint_follower_core.cpp)
target_link_libraries(ff_waypoint_follower
  ${catkin_LIBRARIES}
  ${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudadevrt.a
  ${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudart.so
  )
else()
add_executable(ff_waypoint_follower nodes/ff_waypoint_follower.cpp nodes/ff_waypoint_follower_core.cpp)
target_link_libraries(ff_waypoint_follower  ${catkin_LIBRARIES})
endif(CUDA_FOUND)

#link_directories(/usr/local/lib/zmp/minivan)
#add_executable(ff_waypoint_follower nodes/ff_waypoint_follower.cpp nodes/ff_waypoint_follower_core.cpp nodes/HevCnt.cpp nodes/HevComm.cpp)
#target_link_libraries(ff_waypoint_follower utilityh plannerh simuh CANUSB hevcontrol ${catkin_LIBRARIES})

add_dependencies(ff_waypoint_follower 
runtime_manager_generate_messages_cpp
waypoint_follower_msgs_generate_messages_cpp
map_file_generate_messages_cpp
)
