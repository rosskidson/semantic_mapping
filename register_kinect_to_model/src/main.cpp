/*
 * main.cpp
 *
 * register kinect node
 *
 *  Created on: Oct 30, 2012
 *      Author: rosskidson
 */

#include <ros/ros.h>
#include "register_kinect_to_model/kinect_registration.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "mesh_io");

  KinectRegistration registration_obj;
  registration_obj.registerKinectToModel();
  ros::spin();

  return 0;
}