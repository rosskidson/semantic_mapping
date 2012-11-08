/*
 * mesh_converter.cpp
 *
 *  Created on: Nov 3, 2012
 *      Author: ross
 */

#include "mesh_io/mesh_converter.h"
#include "ros/ros.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>


MeshConverter::MeshConverter ()
{
  ros::NodeHandle n("~");
  service_ = n.advertiseService("load_model_from_file", &MeshConverter::loadModelFromFileService, this);
  ROS_INFO("mesh_io services up and running");
}

MeshConverter::~MeshConverter ()
{
  // TODO Auto-generated destructor stub
}

void MeshConverter::convertPCDtoMesh ()
{
  //placeholder
}

void MeshConverter::convertMeshToPcd()
{
  //placeholder
}

bool MeshConverter::loadModelFromFileService(mesh_io::loadModelFromFile::Request  &req,
    mesh_io::loadModelFromFile::Response &res )
{
  if(req.filename.substr(req.filename.length()-3,req.filename.length())=="pcd")
    pcl::toROSMsg(*(loadPointcloudFromFile(req.filename)), res.pointcloud);
  else
    pcl::toROSMsg(*(loadMeshFromFile(req.filename)), res.pointcloud);
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr MeshConverter::loadMeshFromFile(std::string filename)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(filename,mesh);
  pcl::fromROSMsg(mesh.cloud, *input);
  return input;
}

PointCloudConstPtr MeshConverter::loadPointcloudFromFile(std::string filename)
{
  pcl::PCDReader reader;
  PointCloudPtr input (new PointCloud());
  reader.read (filename, *input);
  return input;
}

