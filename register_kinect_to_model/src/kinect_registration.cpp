/*
 * kinect_registration.cpp
 *
 *  Created on: Nov 4, 2012
 *      Author: kidson
 */

#include "register_kinect_to_model/kinect_registration.h"
#include "register_kinect_to_model/icp_wrapper.h"
#include "pcl_typedefs/pcl_typedefs.h"

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/console.h>

static const std::string FEATURE_EXTRACTOR = "SIFT";
static const std::string FEATURE_DESCRIPTOR = "SIFT";
static const std::string DESCRIPTOR_MATCHER = "Bruteforce";
static const int OUTLIER_REMOVAL_FACTOR = 9; //smaller = remove more outliers
static const bool SHOW_BEST_MATCHES = false;

static const bool SAVE_FEATURES_IMAGE = true;

KinectRegistration::KinectRegistration ()
{
}

KinectRegistration::~KinectRegistration ()
{
  // TODO Auto-generated destructor stub
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
    if (all_matches[i].distance < OUTLIER_REMOVAL_FACTOR * min_dist)
      matches.push_back (all_matches[i]);
  }
}

/*
 * Finds the best matching image in a vector of images for a given query image based on SIFT features
 * Returns the index of the best match
 */
uint KinectRegistration::findMatchingImage (const cv::Mat query_image,
    const std::vector<cv::Mat>& images)
{
  std::vector<cv::KeyPoint> query_keypoints;
  cv::Mat query_descriptors;

  std::vector<std::vector<cv::KeyPoint> > keypoints_vector;
  std::vector<cv::Mat> descriptors_vector;
  std::vector<std::vector<cv::DMatch> > matches_vector;

  // get sift features from input
  getFeatures (query_image, query_keypoints, query_descriptors);
  ROS_INFO_STREAM("num features: " << query_keypoints.size());
  // get sift features from vector
  uint max_matches = 0;
  uint winner_idx = 0;
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
    if(temp_match.size() > 25 && SHOW_BEST_MATCHES)
    {
      ROS_INFO_STREAM("winner matches " << temp_match.size()<< " idx: " << itr - images.begin());
      cv::Mat img_matches;
      cv::drawMatches (query_image, query_keypoints, *itr, temp_keypoints,
          temp_match, img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1),
          std::vector<char> (), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      cv::imshow ("Matches", img_matches);
      cv::moveWindow ("Matches", -10, 0);
      cv::waitKey (0);
    }
    if(temp_match.size() > max_matches)
    {
      max_matches = temp_match.size();
      winner_idx = itr - images.begin();
    }

  }
  ROS_INFO_STREAM("winner idx " << winner_idx);
  return winner_idx;
}

Eigen::Matrix4f KinectRegistration::registerKinectToModel (const PointCloudConstPtr model_cloud_ptr, const PointCloudConstPtr kinect_cloud_ptr,
    cv::Mat kinect_image, std::vector<cv::Mat>& images, std::vector<Eigen::Matrix4f>& transforms )
{
  uint best_image = findMatchingImage (kinect_image, images);

  ICPWrapper icp;
  Eigen::Matrix4f resulting_transform = icp.performICP(kinect_cloud_ptr, model_cloud_ptr,transforms[best_image]);

  return resulting_transform;
}
