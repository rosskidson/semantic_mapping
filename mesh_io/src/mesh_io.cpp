/*
 * MeshIO.cpp
 *
 *  Created on: Nov 17, 2012
 *      Author: kidson
 */

#include "mesh_io/mesh_io.h"

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

MeshIO::MeshIO ()
{
  ros::NodeHandle n ("~");
}

MeshIO::~MeshIO ()
{
}

PointCloudPtr MeshIO::loadMeshFromFile (std::string filename)
{
  PointCloudPtr input (new PointCloud);
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr temp (new pcl17::PointCloud<pcl17::PointXYZRGB>);
  pcl17::PolygonMesh mesh;
  pcl17::io::loadPolygonFile (filename, mesh);
  pcl17::fromROSMsg (mesh.cloud, *temp);
  *input = *temp;
  return input;
}

PointCloudPtr MeshIO::loadPointcloudFromFile (std::string filename)
{
  pcl17::PCDReader reader;
  PointCloudPtr input (new PointCloud);
  reader.read (filename, *input);
  return input;
}

void MeshIO::savePointcloudToFile (const PointCloudConstPtr input_cloud_ptr, std::string filename)
{
  pcl17::PCDWriter writer;
  writer.write(filename, *input_cloud_ptr);
}

cv::Mat MeshIO::loadImageFromFile(std::string filename)
{
  return cv::imread (filename, CV_LOAD_IMAGE_COLOR);
}
/*
 * Iterate through a directory and get all the paths with a certain extension
 */

void MeshIO::getFileListWithExtension (const std::string& input_dir,
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

void MeshIO::loadImagesFromDir (std::string directory, std::vector<cv::Mat>& images)
{
  std::set<std::string> file_list;
  getFileListWithExtension (directory, ".png", file_list);
  for (std::set<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
  {
    cv::Mat cv_image = cv::imread (*itr, CV_LOAD_IMAGE_COLOR);
    images.push_back(cv_image);
  }
}

void MeshIO::loadTransformationsFromDir (std::string directory, std::vector<Eigen::Matrix4f>& transforms)
{
  std::set<std::string> file_list;
  getFileListWithExtension (directory, ".txt", file_list);
  for (std::set<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
    transforms.push_back (extractTransformationFromFile (*itr));
}

// the following file formatting expects output from pcl_kinfu_largeScale

Eigen::Matrix4f MeshIO::extractTransformationFromFile (std::string filename)
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

void MeshIO::loadPointcloudsFromDir (std::string directory, std::vector<PointCloud>& pointclouds)
{
  std::set<std::string> file_list;
  pcl17::PCDReader reader;
  PointCloud temp_pointcloud;
  getFileListWithExtension (directory, ".pcd", file_list);
  for (std::set<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
  {
    reader.read (*itr, temp_pointcloud);
    pointclouds.push_back (temp_pointcloud);
  }
}
