/*
 * kinect_registration.h
 *
 *  Created on: Nov 4, 2012
 *      Author: kidson
 */

#ifndef KINECT_REGISTRATION_H_
#define KINECT_REGISTRATION_H_

#include "register_kinect_to_model/registerKinectToModel.h"
#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"

class KinectRegistration
{
  public:
    KinectRegistration ();
    virtual ~KinectRegistration ();

    void getTransformFromClosestImage();
  private:
    bool registerKinectToModel (register_kinect_to_model::registerKinectToModel::Request &req,
        register_kinect_to_model::registerKinectToModel::Response &res);

    int findMatchingImage (const cv::Mat query_image, const std::vector<cv::Mat>& images);

    void getFeatures (const cv::Mat& input_image,
        std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

    void findMatches (const cv::Mat& source_descriptors,
        const cv::Mat& target_descriptors, std::vector<cv::DMatch>& matches);

    ros::ServiceServer service_;
    ros::NodeHandle nh_;
    int image_counter_;
};

#endif /* KINECT_REGISTRATION_H_ */
