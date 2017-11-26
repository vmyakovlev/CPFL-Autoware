cmake_minimum_required(VERSION 2.8.3)
project(op_planner)

# Get a ros version
execute_process(
	COMMAND rosversion -d
	OUTPUT_VARIABLE ROS_VERSION
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(catkin REQUIRED COMPONENTS
		op_utility
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES  op_planner
   CATKIN_DEPENDS op_utility
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")

set(PLANNERH_SRC
		src/BehaviorStateMachine.cpp
		src/MappingHelpers.cpp
		src/RSPlanner.cpp
		src/GridMap.cpp
		src/MatrixOperations.cpp
		src/TrajectoryCosts.cpp
		src/HMIStateMachine.cpp
		src/PlannerH.cpp
		src/geo_pos_conv.cpp
		src/LocalPlannerH.cpp
		src/PlanningHelpers.cpp
)

# Check CUDA existence
find_package(CUDA)
if (CUDA_FOUND)
INCLUDE(FindCUDA)

## Detect GPU architecture genetation
set(CUDA_CAPABILITY_VERSION_CHECKER
  "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker"
  )

add_custom_target(gpu_arch_version_checker_for_op_planner)
add_dependencies(gpu_arch_version_checker_for_op_planner
  libdpm_ttic			# this checker is defined in libdpm_ttic
  )

execute_process(
  COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
  OUTPUT_VERIABLE CUDA_CAPABILITY_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

if ("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
  set(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
else()
  set(CUDA_ARCH "sm_52")
endif()


function(add_cuda_lib name files cpp_files libraries)
  # Create flag to make cuda enable
  set(CUDA_ENABLE_FLAGS
	"-DCUDA_FOUND"
	)

  # Collect required libraries
  set(lib_list)
  foreach(library_name ${libraries})
  	LIST(APPEND lib_list "-l${library_name}")
  endforeach()

  # Generate object files for each .cu files
  set(obj_list)
  foreach(file ${files})
	# Extract file name from path
	get_filename_component(file_name ${file} NAME)

	# Execute nvv to generate object file
	add_custom_command(
	  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${file_name}.o
	  DEPENDS ${file}
	  COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --blue "Building NVCC Device object ${CMAKE_CURRENT_SOURCE_DIR}/${file}"
	  COMMAND ${CUDA_NVCC_EXECUTABLE} ${CUDA_ENABLE_FLAGS} ${CUDA_NVCC_FLAGS} -I${CMAKE_CURRENT_SOURCE_DIR}/include -I${catkin_INCLUDE_DIRS} --device-c "${CMAKE_CURRENT_SOURCE_DIR}/${file}" -o "${CMAKE_CURRENT_BINARY_DIR}/${file_name}.o" ${lib_list}
	  COMMENT "Building ${CMAKE_CURRENT_SOURCE_DIR}/${file}"
	  VERBATIM
	  )

	# Create list of object files
	LIST(APPEND obj_list ${CMAKE_CURRENT_BINARY_DIR}/${file_name}.o)

  endforeach()

  # Set compile flags and target name
  set(obj_name "${CMAKE_CURRENT_BINARY_DIR}/libcuda_compiled_library.a")

  # Get full path for cpp sources
  set(cpp_path)
  foreach(file ${cpp_files})
	LIST(APPEND cpp_path "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
  endforeach()

  # Link cuda object files into one
  add_custom_target(cuda_linked_object ALL
	COMMAND ${CUDA_NVCC_EXECUTABLE} ${CUDA_NVCC_FLAGS} --device-link ${obj_list} -o cuda_linked_object.o
  	DEPENDS ${obj_list}
  	)

  # Generate object files for each cpp file
  foreach(file ${cpp_files})
	# Extract file name from path
	get_filename_component(file_name ${file} NAME)

	# Execute cpp compiler to generate object file
	add_custom_command(
	  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${file_name}.o
	  DEPENDS ${file}
	  COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --blue "Building c++ Device object ${CMAKE_CURRENT_SOURCE_DIR}/${file}"
	  COMMAND ${CMAKE_CXX_COMPILER} ${CUDA_ENABLE_FLAGS} -std=c++11 -I${CUDA_INCLUDE_DIRS} -I${CMAKE_CURRENT_SOURCE_DIR}/include -I${catkin_INCLUDE_DIRS} -L${CUDA_TOOLKIT_ROOT_DIR}/lib64 -c "${CMAKE_CURRENT_SOURCE_DIR}/${file}" -o "${CMAKE_CURRENT_BINARY_DIR}/${file_name}.o" ${lib_list}
	  COMMENT "Building ${CMAKE_CURRENT_SOURCE_DIR}/${file}"
	  VERBATIM
	  )

	# Create list of object files
	LIST(APPEND obj_list ${CMAKE_CURRENT_BINARY_DIR}/${file_name}.o)

  endforeach()

  # Link cpp files and cuda object files and generate static library
  add_custom_target(linked_library ALL
	COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --red "Linking CXX executable ${name}"
	COMMAND ${CUDA_NVCC_EXECUTABLE} --lib --output-file ${obj_name} ${obj_list} cuda_linked_object.o
	DEPENDS ${obj_list}
	)

  add_dependencies(linked_library cuda_linked_object)
  add_library(${name} STATIC IMPORTED GLOBAL)

  add_custom_target(
	copy_library ALL
	COMMAND cmake -E copy ${obj_name} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/lib${name}.a
	)

  add_dependencies(copy_library linked_library)
  add_dependencies(${name} copy_library)

  set_target_properties(${name}
	PROPERTIES
	IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/lib${name}.a
	)

endfunction()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++11)

set(OP_PLANNER_CUDA_SRC
  src/GPUCalculationHelper.cu
  src/TrajectoryCostsOnGPU.cu
  src/TrajectoryCostsOnGPU_Kernel.cu
  )

add_cuda_lib(${PROJECT_NAME} "${OP_PLANNER_CUDA_SRC}" "${PLANNERH_SRC}" "${catkin_LIBRARIES}")


else()

include_directories(${catkin_INCLUDE_DIRS}
		include
)

## Declare a cpp library
add_library(${PROJECT_NAME}
	    ${PLANNERH_SRC}
)

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
)

endif(CUDA_FOUND)


