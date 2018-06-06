cmake_minimum_required(VERSION 2.8.3)
project(op_utilities)

find_package(catkin REQUIRED COMPONENTS		

	roscpp
  	geometry_msgs  	
  	autoware_msgs
  	map_file
  	op_utility
  	op_planner
  	op_ros_helpers
  	pcl_conversions
)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(X11 REQUIRED)
find_package(Threads REQUIRED)
#find_package(TinyXML REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs map_file op_utility op_planner op_ros_helpers pcl_conversions
  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O0 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")



include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(lib)


add_executable(op_pose_tf nodes/op_pose_tf/op_pose_tf.cpp nodes/op_pose_tf/op_pose_tf_core.cpp)
target_link_libraries(op_pose_tf ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(op_data_logger nodes/op_data_logger/op_data_logger.cpp nodes/op_data_logger/op_data_logger_core.cpp)
target_link_libraries(op_data_logger ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(op_testing_ui 
nodes/op_testing_ui/op_testing_ui.cpp
nodes/op_testing_ui/op_testing_ui_core.cpp
nodes/op_testing_ui/DrawObjBase.cpp
nodes/op_testing_ui/DrawingHelpers.cpp
nodes/op_testing_ui/MainWindowWrapper.cpp
nodes/op_testing_ui/Graph2dBase.cpp
nodes/op_testing_ui/SimulatedController.cpp 
nodes/op_testing_ui/BagTopicPlayer.cpp)

target_link_libraries(op_testing_ui  
		${catkin_LIBRARIES}		
		${OpenCV_LIBRARIES}
		${OPENGL_LIBRARIES}
		${GLUT_LIBRARIES}
		${GLEW_LIBRARIES}
		${X11_LIBRARIES}
		${TinyXML_LIBRARIES}
)


add_dependencies(op_pose_tf op_data_logger op_testing_ui ${catkin_EXPORTED_TARGETS})

