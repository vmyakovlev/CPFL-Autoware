cmake_minimum_required(VERSION 2.8.3)
project(testing_ui)

find_package(catkin REQUIRED COMPONENTS		

	roscpp
  	geometry_msgs  	
  	autoware_msgs
  	map_file
  	op_utility
  	op_planner
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
  CATKIN_DEPENDS roscpp geometry_msgs map_file op_utility op_planner
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

link_directories(lib)
add_executable(testing_ui 
nodes/testing_ui.cpp
nodes/AlternativeVisualizer.cpp
nodes/DrawObjBase.cpp
nodes/DrawingHelpers.cpp
nodes/glm.cpp
nodes/MainWindowWrapper.cpp
nodes/Graph2dBase.cpp
nodes/SimulatedController.cpp)

target_link_libraries(testing_ui  
		${catkin_LIBRARIES}		
		${OpenCV_LIBRARIES}
		${OPENGL_LIBRARIES}
		${GLUT_LIBRARIES}
		${GLEW_LIBRARIES}
		${X11_LIBRARIES}
		${TinyXML_LIBRARIES}
)


add_dependencies(testing_ui 
${catkin_EXPORTED_TARGETS}
)
