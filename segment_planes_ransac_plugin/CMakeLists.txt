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

include_directories("/work/kidson/pcl-trunk/common/include")
include_directories("/work/kidson/pcl-trunk/segmentation/include")
include_directories("/work/kidson/pcl-trunk/geometry/include")
include_directories("/work/kidson/pcl-trunk/features/include")
include_directories("/work/kidson/pcl-trunk/features/include")
include_directories("/work/kidson/pcl-trunk/search/include")
include_directories("/work/kidson/pcl-trunk/kdtree/include")
include_directories("/work/kidson/pcl-trunk/octree/include")
include_directories("/work/kidson/pcl-trunk/build/include")
#link_directories("/work/kidson/pcl-trunk/build/lib")

#common commands for building c++ executables and libraries
rosbuild_add_library(segment_planes_ransac_plugin src/plane_segmentation_ransac.cpp)
target_link_libraries(segment_planes_ransac_plugin /work/kidson/pcl-trunk/build/lib/libpcl_segmentation.so /work/kidson/pcl-trunk/build/lib/libpcl_features.so)# /work/kidson/pcl-trunk/build/lib/libpcl_search.so)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

