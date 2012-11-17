/*
 * icp_wrapper.cpp
 *
 *  Created on: Nov 8, 2012
 *      Author: kidson
 */

#include "register_kinect_to_model/icp_wrapper.h"
#include <ros/console.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

ICPWrapper::ICPWrapper ()
{
  // TODO Auto-generated constructor stub
  //icp.setRANSACOutlierRejectionThreshold (1000000.0);
  //icp.setMaximumIterations (1);
  //icp.setMaxCorrespondenceDistance (4.0);
}

ICPWrapper::~ICPWrapper ()
{
  // TODO Auto-generated destructor stub
}

Eigen::Matrix4f ICPWrapper::performICP (PointCloudConstPtr source_cloud_ptr,
    PointCloudConstPtr target_cloud_ptr, Eigen::Matrix4f initial_transform)
{
  ROS_INFO("Performing ICP...");
  PointCloud source_cloud_transformed;
  icp_.setInputCloud (source_cloud_ptr);
  icp_.setInputTarget (target_cloud_ptr);
  icp_.align (source_cloud_transformed, initial_transform);
  ROS_INFO_STREAM ("ICP has converged: " << icp_.hasConverged() << " score: "
      << icp_.getFitnessScore () << "\n Transformation: \n" << icp_.getFinalTransformation ());
  return icp_.getFinalTransformation ();
}
