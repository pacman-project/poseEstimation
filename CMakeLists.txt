cmake_minimum_required(VERSION 2.8.3)
project(uibk_pose_estimation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(PCL 1.7 REQUIRED )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(pose_estimation SHARED
   src/ParametersPoseEstimation.cpp 
   src/I_SegmentedObjects.cpp
)

add_executable(execution_control
	src/executionControl.cpp
)

add_dependencies(execution_control pose_estimation)

## Specify libraries to link a library or executable target against
target_link_libraries(pose_estimation
   ${boost_LIBRARIES} ${PCL_LIBRARIES} pcl_3d_rec_framework
)
target_link_libraries(execution_control
   pose_estimation
)