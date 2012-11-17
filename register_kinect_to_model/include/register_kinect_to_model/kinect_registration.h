/*
 * kinect_registration.h
 *
 *  Created on: Nov 4, 2012
 *      Author: kidson
 */

#ifndef KINECT_REGISTRATION_H_
#define KINECT_REGISTRATION_H_

#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include "pcl_typedefs/pcl_typedefs.h"
#include <Eigen/Core>

class KinectRegistration
{
  public:
    KinectRegistration ();
    virtual ~KinectRegistration ();

    Eigen::Matrix4f registerKinectToModel (const PointCloudConstPtr model_cloud_ptr, const PointCloudConstPtr kinect_cloud_ptr,
        cv::Mat kinect_image, std::vector<cv::Mat>& images, std::vector<Eigen::Matrix4f>& transforms );

  private:
    uint findMatchingImage (const cv::Mat query_image, const std::vector<cv::Mat>& images);

    void getFeatures (const cv::Mat& input_image,
        std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

    void findMatches (const cv::Mat& source_descriptors,
        const cv::Mat& target_descriptors, std::vector<cv::DMatch>& matches);

    int image_counter_;
};

#endif /* KINECT_REGISTRATION_H_ */
