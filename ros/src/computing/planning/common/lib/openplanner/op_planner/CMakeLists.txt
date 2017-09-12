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

# include_directories(${catkin_INCLUDE_DIRS}
# 		include
# )

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

# ## Set compiler option
# # list(APPEND CUDA_NVCC_FLAGS "-std=c++11" "-arch=${CUDA_ARCH}" "--compiler-options -fPIC" "-rdc=true" "--shared" ${CUDA_NVCC_FLAGS})
# # set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -std=c++11; -arch=${CUDA_ARCH}; -rdc=true;)
# # Debug用
# # list(APPEND CUDA_NVCC_FLAGS "-std=c++11" "-arch=${CUDA_ARCH}" "--compiler-options -fPIC" "-rdc=true" "--shared" "-DTHRUST_DEBUG" ${CUDA_NVCC_FLAGS})
# set(CUDA_SEPARABLE_COMPILATION ON) # これの位置はここでいいっぽい気がする
# # set(BUILD_SHARED_LIBS OFF)
# # set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF)
# # set(CUDA_PROPAGATE_HOST_FLAG OFF)

# ## Source file list containing CUDA code
# file(GLOB OP_PLANNER_SRC "src/*.cpp")
# file(GLOB OP_PLANNER_HEADER "include/*.h")
# file(GLOB OP_PLANNER_CUDA_SRC "src/*.cu")

# ## Declare a library using CUDA
# include_directories(
# 		include
# 		${CUDA_INCLUDE_DIRS}
# 		${catkin_INCLUDE_DIRS}
# )

# # cuda_include_directories(
# # 		include
# # 		${CUDA_INCLUDE_DIRS}
# # 		${catkin_INCLUDE_DIRS}
# # )


# # Create object files for each .cu files and link them
# ## Note: "cuda_add_library" is not used here because the function generate "unknown error"
# # set(CUDA_COMPILE_OPTIONS "-arch=${CUDA_ARCH} -rdc=true --compiler-options -fPIC -std=c++11")
# set(CUDA_COMPILE_OPTIONS "-arch=${CUDA_ARCH} -dc --compiler-options -fPIC -std=c++11")

# # cuda_compile(DEVICE_OBJECTS
# #   ${OP_PLANNER_CUDA_SRC}
# #   ${OP_PLANNER_CUDA_HEADER}
# #   ${OP_PLANNER_SRC}
# #   OPTIONS ${CUDA_COMPILE_OPTIONS}
# #   )
# CUDA_WRAP_SRCS(${PROJECT_NAME}
#   OBJ
#   DEVICE_OBJECTS
#   ${OP_PLANNER_CUDA_SRC} ${OP_PLANNER_CUDA_HEADER}
#   OPTIONS ${CUDA_COMPILE_OPTIONS}
#   )

# cuda_compute_separable_compilation_object_file_name(LINK_OBJECTS
#   ${PROJECT_NAME}
#   ${DEVICE_OBJECTS}
#   )

# cuda_link_separable_compilation_objects(${LINK_OBJECTS}
#   ${PROJECT_NAME}
#   ${COMPLE_OPTIONS}
#   ${DEVICE_OBJECTS}
#   )

# ########################################
# ########################################
# # 元々のコードではSHARED（.so）が生成される
# ########################################
# ########################################
# add_library(${PROJECT_NAME} SHARED
#   ${DEVCIE_OBJECTS}
#   ${LINK_OBJECTS}
#   ${OP_PLANNER_SRC}
#   # ${OP_PLANNER_CUDA_SRC}		#これがないと各.cuに対して.oファイルが生成されない→-fPICつけて再コンパイルしろってエラーが出る
#   )

# ##############################################################################33
# # これらをつけたところで、fPICをつけろってのは変わらない。謎
# ##############################################################################33
# # set_target_properties(${PROJECT_NAME}
# #   PROPERTIES COMPILE_FLAGS "-fPIC"
# #   )

# # set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
# ##############################################################################33
# ##############################################################################33

# # cuda_add_library(${PROJECT_NAME}
# # 	${OP_PLANNER_SRC}
# # 	${OP_PLANNER_HEADER}
# # 	${OP_PLANNER_CUDA_SRC}
# # 	# SHARED
# # 	OPTIONS --compiler-options -fPIC
# # )

# find_library(CUDA_DEVICE_RUNTIME_LIBRARY cudadevrt ${CUDA_TOOLKIT_ROOT_DIR}/lib64)
# # set(MYLIB -Wl,--whole-archive ${CUDA_DEVICE_RUNTIME_LIBRARY} -Wl,--no-whole-archive)

# target_link_libraries(${PROJECT_NAME}
# 		${catkin_LIBRARIES}
# 		${OpenCV_LIBS}
# 		${CUDA_LIBRARIES}
# 		${CUDA_DEVICE_RUNTIME_LIBRARY}
# 		# ${MYLIB}
# )

# set_target_properties(${PROJECT_NAME}
#   PROPERTIES
#   LINKER_LANGUAGE ${CUDA_C_OR_CXX}
#   )

function(add_cuda_exe_lib name files libraries is_lib)
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
	  COMMAND ${CUDA_NVCC_EXECUTABLE} ${CUDA_NVCC_FLAGS} -I${CMAKE_CURRENT_SOURCE_DIR}/include -I${catkin_INCLUDE_DIRS} -dc "${CMAKE_CURRENT_SOURCE_DIR}/${file}" -o "${CMAKE_CURRENT_BINARY_DIR}/${file_name}.o"
	  COMMENT "Building ${CMAKE_CURRENT_SOURCE_DIR}/${file}"
	  VERBATIM
	  )

	# Create list of object files
	LIST(APPEND obj_list ${CMAKE_CURRENT_BINARY_DIR}/${file_name}.o)

  endforeach()

  # Collect required libraries
  set(lib_list)
  LIST(APPEND lib_list "-lcudadevrt") # "-lcudadevrt" is required for dynamic parallelism
  foreach(library_name ${libraries})
	LIST(APPEND lib_list "${library_name}")
  endforeach()

  # Set compile flags and target name according to "is_lib" flag
  set(flags ${CUDA_NVCC_FLAGS})
  if (is_lib)
	LIST(APPEND flags "-dlink")
	set(obj_name "${CMAKE_CURRENT_BINARY_DIR}/${name}.so")
  else()
	set(obj_name "${CMAKE_CURRENT_BINARY_DIR}/${name}")
  endif()

  # Generate target
  add_custom_target(${name} ALL
  # add_custom_target(cuda_compiled_library ALL
	COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --red "Linking CXX executable ${name}"
	COMMAND ${CUDA_NVCC_EXECUTABLE} ${flags} ${obj_list} ${lib_list} -o ${obj_name}
	DEPENDS ${obj_list}
	COMMENT "Linking ${name}"
	)

  # add_dependencies(${PROJECT_NAME}
  # 	cuda_compiled_library
  # 	)

endfunction()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++11)

set(OP_PLANNER_CUDA_SRC
  src/GPUCalculationHelper.cu
  src/TrajectoryCostsOnGPU.cu
  src/TrajectoryCostsOnGPU_Kernel.cu
  )

add_cuda_exe_lib(${PROJECT_NAME} "${OP_PLANNER_CUDA_SRC}" "${catkin_LIBRARIES}" ON)


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


