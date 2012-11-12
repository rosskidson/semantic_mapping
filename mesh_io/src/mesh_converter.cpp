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
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ros/conversions.h>

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

PointCloudConstPtr MeshConverter::loadMeshFromFile (std::string filename)
{
  PointCloudPtr input (new PointCloud);
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile (filename, mesh);
  pcl::fromROSMsg (mesh.cloud, *input);
  return input;
}

PointCloudConstPtr MeshConverter::loadPointcloudFromFile (std::string filename)
{
  pcl::PCDReader reader;
  PointCloudPtr input (new PointCloud);
  reader.read (filename, *input);
  return input;
}
/*
 * Iterate through a directory and get all the paths with a certain extension
 */

void MeshConverter::getFileListWithExtension (const std::string& input_dir,
    const std::string& input_ext, std::set<std::string>& file_list)
{
  std::ifstream input_stream;
  namespace fs = boost::filesystem;

  fs::path path (input_dir);

  fs::directory_iterator itr (path), eod;

  BOOST_FOREACH(fs::path const &file_path, std::make_pair(itr, eod))
{  if(is_regular_file(file_path))
  {
    //ROS_INFO_STREAM("file:" << file_path.c_str() << " ext " << file_path.extension().c_str());
    if(file_path.extension().string() == input_ext)
    file_list.insert(file_path.string());
  }
}
}

void MeshConverter::loadImagesFromDir (std::string directory, std::vector<cv::Mat>& images)
{
  std::set<std::string> file_list;
  getFileListWithExtension (directory, ".png", file_list);
  for (std::set<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
  {
    cv::Mat cv_image = cv::imread (*itr, CV_LOAD_IMAGE_COLOR);
    images.push_back(cv_image);
  }
}

void MeshConverter::loadTransformationsFromDir (std::string directory, std::vector<Eigen::Matrix4f>& transforms)
{
  std::set<std::string> file_list;
  getFileListWithExtension (directory, ".txt", file_list);
  for (std::set<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
    transforms.push_back (extractTransformationFromFile (*itr));
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

void MeshConverter::loadPointcloudsFromDir (std::string directory, std::vector<PointCloud>& pointclouds)
{
  std::set<std::string> file_list;
  pcl::PCDReader reader;
  PointCloud temp_pointcloud;
  getFileListWithExtension (directory, ".pcd", file_list);
  for (std::set<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
  {
    reader.read (*itr, temp_pointcloud);
    pointclouds.push_back (temp_pointcloud);
  }
}
