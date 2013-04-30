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

# PCL ####################
find_package(PCL 1.6.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# eIGEN ####################
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# VTK ####################
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
#MESSAGE("OPENCV DIR--------------------")
#MESSAGE(${VTK_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
rosbuild_add_library(mesh_io src/mesh_converter.cpp src/mesh_io.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(mesh_io src/main.cpp src/mesh_converter.cpp)
target_link_libraries(mesh_io boost_filesystem boost_system ${PCL_LIBRARIES})
