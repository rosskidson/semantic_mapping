/*
 * axis_alignment.cpp
 *
 *  Created on: Nov 18, 2012
 *      Author: kidson
 */

#include "align_principle_axis/axis_alignment.h"

#include "pcl_typedefs/pcl_typedefs.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include <ros/console.h>

AxisAlignment::AxisAlignment ()
{
  // TODO Auto-generated constructor stub

}

AxisAlignment::~AxisAlignment ()
{
  // TODO Auto-generated destructor stub
}

void AxisAlignment::alignCloudPrincipleAxis (const PointCloudConstPtr input_cloud_ptr,
    const Eigen::Matrix4f& inital_guess, PointCloudPtr output_cloud_ptr,
    Eigen::Matrix4f transform_output)
{
  PointCloudPtr rotated_cloud (new PointCloud);
  ROS_INFO_STREAM("transform to be used: \n" << inital_guess);
  transformPointCloud (*input_cloud_ptr, *rotated_cloud, inital_guess);

  //assume the model is approx correct, i.e. z is height
  // find the largest plane perpendicular to z axis and use this to align cloud
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  seg.setAxis(Eigen::Vector3f(0,0,1));
  seg.setEpsAngle(10.0 * (M_PI/180)); //radians
  seg.setInputCloud (rotated_cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1]
      << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
  double rotate =  /*acos*/coefficients->values[2];
  Eigen::Vector3f axis_to_rotate_about(-coefficients->values[1], -coefficients->values[0]);
  Eigen::Matrix4f rotation_matrix; // make rotation mat from rotate and axis to rotate about

  pcl::ExtractIndices<PointType> eifilter;
   eifilter.setInputCloud (rotated_cloud);
   eifilter.setIndices (inliers);
   eifilter.filter (*output_cloud_ptr);
}
