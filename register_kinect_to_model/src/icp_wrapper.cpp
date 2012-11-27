/*
 * icp_wrapper.cpp
 *
 *  Created on: Nov 8, 2012
 *      Author: kidson
 */

#include "register_kinect_to_model/icp_wrapper.h"
#include <ros/console.h>
#include <pcl17/point_types.h>
#include <pcl17/registration/icp.h>
#include <pcl17/filters/voxel_grid.h>

ICPWrapper::ICPWrapper ()
{
  //icp_.setRANSACOutlierRejectionThreshold (1000000.0);
  icp_.setMaximumIterations (50);
  //icp_.setRANSACOutlierRejectionThreshold(0.04);
  //icp_.setMaxCorrespondenceDistance (4.0);
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
  icp_.setInputCloud (downsampleCloud(source_cloud_ptr));
  icp_.setInputTarget (downsampleCloud(target_cloud_ptr));

  pcl17::console::setVerbosityLevel (pcl17::console::L_DEBUG);

  icp_.align (source_cloud_transformed, initial_transform);
  ROS_INFO_STREAM ("ICP has converged: " << icp_.hasConverged() << " score: "
      << icp_.getFitnessScore () << "\n Transformation: \n" << icp_.getFinalTransformation ());
  return icp_.getFinalTransformation ();
}

PointCloudConstPtr ICPWrapper::downsampleCloud (PointCloudConstPtr input)
{
  const double voxel_size = 0.01;
  PointCloudPtr cloud_filtered (new PointCloud);
  pcl17::VoxelGrid<PointType> downsampler;
  downsampler.setInputCloud (input);
  downsampler.setLeafSize (voxel_size, voxel_size, voxel_size);
  downsampler.filter (*cloud_filtered);
  return cloud_filtered;
}
