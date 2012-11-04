/*
 * kinect_registration.cpp
 *
 *  Created on: Nov 4, 2012
 *      Author: kidson
 */

#include "../include/register_kinect_to_model/kinect_registration.h"

KinectRegistration::KinectRegistration ()
{
  ros::NodeHandle n("~");
  service_ = n.advertiseService("register_kinect_to_model", &KinectRegistration::registerKinectToModel, this);
  ROS_INFO("register kinect service up and running");
}

KinectRegistration::~KinectRegistration ()
{
  // TODO Auto-generated destructor stub
}

bool KinectRegistration::registerKinectToModel(register_kinect_to_model::registerKinectToModel::Request  &req,
    register_kinect_to_model::registerKinectToModel::Response &res )
{
  return true;
}
