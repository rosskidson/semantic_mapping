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
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//file reading stuff
// ################ TO BE REMOVED##################
// when you remove this, don't forgot the linking stuff as well in cmakelists
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>

static const std::string FEATURE_EXTRACTOR = "SIFT";
static const std::string FEATURE_DESCRIPTOR = "SIFT";
static const std::string DESCRIPTOR_MATCHER = "Bruteforce";

static const bool SAVE_FEATURES_IMAGE = true;

KinectRegistration::KinectRegistration () :
  nh_ ("~"), image_counter_ (0)
{
  service_ = nh_.advertiseService ("register_kinect_to_model",
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
{  if(is_regular_file(file_path))
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
      input_stream.close();
      transforms.push_back(trafo);
    }
  }
}
}

void KinectRegistration::getFeatures (const cv::Mat& input_image,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
  // convert to black and white
  cv::Mat image_greyscale;
  cvtColor (input_image, image_greyscale, CV_RGB2GRAY);

  //detect features
  cv::FeatureDetector* detector;
  if (FEATURE_EXTRACTOR == "SIFT")
    detector = new cv::SiftFeatureDetector;
  else
    detector = new cv::SurfFeatureDetector (400);
  detector->detect (image_greyscale, keypoints);

  //extract features
  cv::DescriptorExtractor* extractor;

  if (FEATURE_DESCRIPTOR == "SIFT")
    extractor = new cv::SiftDescriptorExtractor;
  else
    extractor = new cv::SurfDescriptorExtractor;
  extractor->compute (image_greyscale, keypoints, descriptors);

  if (SAVE_FEATURES_IMAGE)
  {
    cv::Mat output;
    cv::drawKeypoints (image_greyscale, keypoints, output);
    std::stringstream result;
    result << "sift_result" << image_counter_++ << ".jpg";
    cv::imwrite (result.str (), output);
  }
}

void KinectRegistration::findMatches (const cv::Mat& source_descriptors,
    const cv::Mat& target_descriptors, std::vector<cv::DMatch>& matches)
{
  std::vector<cv::DMatch> all_matches;
  cv::DescriptorMatcher* matcher;
  if (DESCRIPTOR_MATCHER == "FLANN")
    matcher = new cv::FlannBasedMatcher;
  else
    matcher = new cv::BFMatcher (cv::NORM_L1, false);
  matcher->match (source_descriptors, target_descriptors, all_matches);

  // Outlier detection
  double max_dist = 0;
  double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (uint i = 0; i < all_matches.size (); i++)
  {
    double dist = all_matches[i].distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  //-- Find only "good" matches (i.e. whose distance is less than x*min_dist )
  //-- PS.- radiusMatch can also be used here.
  for (uint i = 0; i < all_matches.size (); i++)
  {
    if (all_matches[i].distance < 10 * min_dist)
      matches.push_back (all_matches[i]);
  }
}

/*
 * Finds the best matching image in a vector of images for a given query image based on SIFT features
 * Returns the number of matches
 */
int KinectRegistration::findMatchingImage (const cv::Mat query_image,
    const std::vector<cv::Mat>& images)
{
  std::vector<cv::KeyPoint> query_keypoints;
  cv::Mat query_descriptors;

  std::vector<std::vector<cv::KeyPoint> > keypoints_vector;
  std::vector<cv::Mat> descriptors_vector;
  std::vector<std::vector<cv::DMatch> > matches_vector;

  // get sift features from input
  getFeatures (query_image, query_keypoints, query_descriptors);
  ROS_INFO_STREAM("no features " << query_keypoints.size());
  // get sift features from vector
  uint max_matches = 0;
  uint winner = 0;
  for (std::vector<cv::Mat>::const_iterator itr = images.begin (); itr != images.end (); itr++)
  {
    std::vector<cv::KeyPoint> temp_keypoints;
    cv::Mat temp_descriptors;
    std::vector<cv::DMatch> temp_match;

    getFeatures (*itr, temp_keypoints, temp_descriptors);
    findMatches (query_descriptors, temp_descriptors, temp_match);

    keypoints_vector.push_back (temp_keypoints);
    descriptors_vector.push_back (temp_descriptors);
    matches_vector.push_back (temp_match);
    //ROS_INFO_STREAM("matches " << temp_match.size());
    if(temp_match.size() > 25)
    {
      ROS_INFO_STREAM("winner matches " << temp_match.size());
      cv::Mat img_matches;
      cv::drawMatches (query_image, query_keypoints, *itr, temp_keypoints,
          temp_match, img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1),
          std::vector<char> (), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      cv::imshow ("Matches", img_matches);
      cv::moveWindow ("Matches", -10, 0);
      cv::waitKey (0);
    }
  }



  // find the best match
  // ????

  return 0;
}

void KinectRegistration::registerKinectToModel ()
{
  //load all the data.  This is a bunch of hardcoded stuff that will be moved out once this package is working

  std::vector<cv::Mat> images;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;
  readImagesTransformsFromDirectory (images, transforms);

  cv::Mat kinect_img = cv::imread (
      "/work/kidson/meshes/cabinet_scan_3/frames_to_register/image_2.png", CV_LOAD_IMAGE_COLOR);
  findMatchingImage (kinect_img, images);

}
