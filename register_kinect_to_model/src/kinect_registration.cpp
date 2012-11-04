/*
 * kinect_registration.cpp
 *
 *  Created on: Nov 4, 2012
 *      Author: kidson
 */

#include "../include/register_kinect_to_model/kinect_registration.h"

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//file reading stuff
// ################ TO BE REMOVED##################
// when you remove this, don't forgot the linking stuff as well in cmakelists
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>

KinectRegistration::KinectRegistration ()
{
  ros::NodeHandle n ("~");
  service_ = n.advertiseService ("register_kinect_to_model",
      &KinectRegistration::registerKinectToModel, this);
  ROS_INFO("register kinect service up and running");
}

KinectRegistration::~KinectRegistration ()
{
  // TODO Auto-generated destructor stub
}

bool KinectRegistration::registerKinectToModel (
    register_kinect_to_model::registerKinectToModel::Request &req,
    register_kinect_to_model::registerKinectToModel::Response &res)
{
  return true;
}

void readImagesTransformsFromDirectory (std::vector<cv::Mat>& images, std::vector<Eigen::Matrix4f,
    Eigen::aligned_allocator<Eigen::Matrix4f> >& transforms)
{
        std::ifstream input_stream;
  namespace fs = boost::filesystem;

  fs::path targetDir ("/work/kidson/meshes/cabinet_scan_3/KinFuSnapshots");

  fs::directory_iterator it (targetDir), eod;

  BOOST_FOREACH(fs::path const &file_path, std::make_pair(it, eod))
  {
    if(is_regular_file(file_path))
    {
      ROS_INFO_STREAM("file:" << file_path.c_str() << " ext " << file_path.extension().c_str());
      if(file_path.extension().string() == ".png")
        images.push_back(cv::imread(file_path.string(), CV_LOAD_IMAGE_COLOR));
      if(file_path.extension().string() == ".txt")
      {
        // the following file formatting expects output from pcl_kinfu_largeScale
        Eigen::Matrix4f trafo;
        input_stream.open (file_path.c_str());
        std::string abcd;
        input_stream >> abcd;   // TVector
        for (int row = 0; row < 3; row++)
          input_stream >> trafo (row, 3);
        input_stream >> abcd;   // RMatrix
        for (int row = 0; row < 3; row++)
          for (int col = 0; col < 3; col++)
            input_stream >> trafo (row, col);
        for (int col = 0; col < 3; col++)
          trafo (3, col) = 0;
        trafo (3, 3) = 1;
        input_stream.close();
        transforms.push_back(trafo);
      }
    }
  }
}

void KinectRegistration::registerKinectToModel ()
{
  //load all the data.  This is a bunch of hardcoded stuff that will be moved out once this package is working

  std::vector<cv::Mat> images;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;
  readImagesTransformsFromDirectory (images,transforms);
  //image = imread(argv[1], CV_LOAD_IMAGE_COLOR);


}
