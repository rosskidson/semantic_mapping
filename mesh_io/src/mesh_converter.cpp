/*
 * mesh_converter.cpp
 *
 *  Created on: Nov 3, 2012
 *      Author: ross
 */

#include "mesh_io/mesh_converter.h"

//ros
#include "ros/ros.h"
//pcl
#include <pcl17/io/ply_io.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/io/vtk_lib_io.h>
#include <pcl17/ros/conversions.h>

//opencv
#include <opencv2/highgui/highgui.hpp>

//file reading stuff
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>

MeshConverter::MeshConverter ()
{
  ros::NodeHandle n ("~");
}

MeshConverter::~MeshConverter ()
{
  // TODO Auto-generated destructor stub
}

void MeshConverter::convertPCDtoMesh ()
{
  //placeholder
}

void MeshConverter::convertMeshToPcd ()
{
  //placeholder
}
