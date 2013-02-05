cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

find_package(PCL 1.6 REQUIRED PATHS )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
message("------------------pcl incls " ${PCL_INCLUDE_DIRS})
message("------------------pcl libs " ${PCL_COMMON_LIBRARIES})
message("------------------pcl io libs " ${PCL_IO_LIBRARIES})
#add_definitions(${PCL_DEFINITIONS})

#include_directories(/opt/ros/groovy/include/pcl17-1.6)

# eIGEN ####################
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(kinect_snapshot src/kinect_capture_frame.cpp)
rosbuild_add_executable(kinect_snapshot_service src/kinect_capture_frame_service.cpp)
target_link_libraries(kinect_snapshot_service ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(kinect_snapshot ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
