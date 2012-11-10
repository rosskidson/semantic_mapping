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
#include <pcl/ros/conversions.h>

#include <opencv2/highgui/highgui.hpp>
// opencv -> ROS -> opencv
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//file reading stuff
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>

MeshConverter::MeshConverter ()
{
  ros::NodeHandle n ("~");
  service_ = n.advertiseService ("load_model_from_file", &MeshConverter::loadModelFromFileService,
      this);
  service_images_ = n.advertiseService ("load_images_from_dir",
      &MeshConverter::loadImagesFromDirService, this);
  service_transforms_ = n.advertiseService ("load_transforms_from_dir",
      &MeshConverter::loadTransformationsFromDirService, this);
  service_pointclouds_ = n.advertiseService ("load_pointclouds_from_dir",
      &MeshConverter::loadPointcloudsFromDirService, this);
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

void MeshConverter::convertMeshToPcd ()
{
  //placeholder
}

bool MeshConverter::loadModelFromFileService (mesh_io::loadModelFromFile::Request &req,
    mesh_io::loadModelFromFile::Response &res)
{
  if (req.filename.substr (req.filename.length () - 3, req.filename.length ()) == "pcd")
    pcl::toROSMsg (* (loadPointcloudFromFile (req.filename)), res.pointcloud);
  else
    pcl::toROSMsg (* (loadMeshFromFile (req.filename)), res.pointcloud);
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr MeshConverter::loadMeshFromFile (std::string filename)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile (filename, mesh);
  pcl::fromROSMsg (mesh.cloud, *input);
  return input;
}

PointCloudConstPtr MeshConverter::loadPointcloudFromFile (std::string filename)
{
  pcl::PCDReader reader;
  PointCloudPtr input (new PointCloud ());
  reader.read (filename, *input);
  return input;
}
/*
 * Iterate through a directory and get all the paths with a certain extension
 */

void MeshConverter::getFileListWithExtension (const std::string& input_dir,
    const std::string& input_ext, std::vector<std::string>& file_list)
{
  std::ifstream input_stream;
  namespace fs = boost::filesystem;

  fs::path path (input_dir);

  fs::directory_iterator itr (path), eod;

  BOOST_FOREACH(fs::path const &file_path, std::make_pair(itr, eod))
{  if(is_regular_file(file_path))
  {
    ROS_INFO_STREAM("file:" << file_path.c_str() << " ext " << file_path.extension().c_str());
    if(file_path.extension().string() == input_ext)
    file_list.push_back(file_path.string());
  }
}
}

bool MeshConverter::loadImagesFromDirService (mesh_io::loadImagesFromDir::Request &req,
    mesh_io::loadImagesFromDir::Response &res)
{
  std::vector<std::string> file_list;
  getFileListWithExtension (req.directory_name, ".png", file_list);
  for (std::vector<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
  {
    cv::Mat cv_image = cv::imread (*itr, CV_LOAD_IMAGE_COLOR);
    cv_bridge::CvImage img_msg;
    //out_msg.header   =
    img_msg.encoding = sensor_msgs::image_encodings::RGB8;
    img_msg.image = cv_image;
    res.images.push_back (*img_msg.toImageMsg ());
  }
  return true;
}

bool MeshConverter::loadTransformationsFromDirService (
    mesh_io::loadTransformationsFromDir::Request &req, mesh_io::loadTransformationsFromDir::Response &res)
{
  std::vector<std::string> file_list;
  getFileListWithExtension (req.directory_name, ".txt", file_list);
  for (std::vector<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
    extractTransformationFromFile (*itr);
  return true;
}

// the following file formatting expects output from pcl_kinfu_largeScale

Eigen::Matrix4f MeshConverter::extractTransformationFromFile (std::string filename)
{
  std::fstream input_stream;
  Eigen::Matrix4f trafo;
  input_stream.open (filename.c_str ());
  std::string abcd;
  input_stream >> abcd; // TVector
  for (int row = 0; row < 3; row++)
    input_stream >> trafo (row, 3);
  input_stream >> abcd; // RMatrix
  for (int row = 0; row < 3; row++)
    for (int col = 0; col < 3; col++)
      input_stream >> trafo (row, col);
  for (int col = 0; col < 3; col++)
    trafo (3, col) = 0;
  trafo (3, 3) = 1;
  input_stream.close ();
  return trafo;
}

bool MeshConverter::loadPointcloudsFromDirService (mesh_io::loadPointcloudsFromDir::Request &req,
    mesh_io::loadPointcloudsFromDir::Response &res)
{
  std::vector<std::string> file_list;
  pcl::PCDReader reader;
  PointCloud temp_pointcloud;
  sensor_msgs::PointCloud2 temp_msg;
  getFileListWithExtension (req.directory_name, ".pcd", file_list);
  for (std::vector<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
  {
    reader.read (*itr, temp_pointcloud);
    pcl::toROSMsg(temp_pointcloud,temp_msg);
    res.pointclouds.push_back (temp_msg);
  }
  return true;
}
