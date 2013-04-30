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
#rosbuild_gensrv()

# eIGEN ####################
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# PCL ####################
find_package(PCL 1.6.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
rosbuild_add_library(pcl_tools src/pcl_tools.cpp)
target_link_libraries(pcl_tools ${PCL_LIBRARIES})
#rosbuild_add_boost_directories()
rosbuild_add_executable(vox_filter src/vox_filter.cpp)
rosbuild_link_boost(vox_filter system)
target_link_libraries(vox_filter pcl_tools)

rosbuild_add_executable(transform_cloud src/transform_cloud.cpp)
rosbuild_link_boost(transform_cloud system)
target_link_libraries(transform_cloud pcl_tools)

rosbuild_add_executable(calculate_normals src/calculate_normals.cpp)
rosbuild_link_boost(calculate_normals system)
target_link_libraries(calculate_normals pcl_tools)



