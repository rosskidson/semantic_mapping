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

# VTK ####################
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
MESSAGE("vtk lib--------------------")
MESSAGE(${VTK_LIBRARY_DIRS})
#SET(VTK_LIBRARIES vtkalglib vtkCharts vtkCommon vtkDICOMParser vtkexoIIc vtkFiltering vtkftgl vtkGenericFiltering vtkGeovis vtkGraphics vtkHybrid vtkImaging vtkInfovis vtkIO vtkmetaio vtkParallel vtkproj4 vtkQtChart vtkRendering vtksqlite vtksys vtkverdict vtkViews vtkVolumeRendering vtkWidgets)
SET(VTK_LIBRARIES vtkCommon vtkFiltering)

rosbuild_find_ros_package(mesh_io)
link_directories(${mesh_io_PACKAGE_PATH}/lib)

rosbuild_find_ros_package(box_filter)
link_directories(${box_filter_PACKAGE_PATH}/lib)

set(SEMANTIC_MAPPING_LIBRARIES mesh_io box_filter)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(main src/main.cpp src/visualization.cpp src/parameter_server.cpp)
rosbuild_link_boost(main thread)
target_link_libraries(main ${VTK_LIBRARIES} ${SEMANTIC_MAPPING_LIBRARIES})
