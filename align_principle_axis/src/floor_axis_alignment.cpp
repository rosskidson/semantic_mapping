/*
 * floor_axis_alignment.cpp
 *
 *  Created on: 20/11/2012
 *      Author: ross
 */

#include "align_principle_axis/floor_axis_alignment.h"
#include <pcl17/ModelCoefficients.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/filters/extract_indices.h>
#include <pcl17/common/transforms.h>

#include <pluginlib/class_list_macros.h>

//Declare the plane segmentation as a segmentation class
PLUGINLIB_DECLARE_CLASS(align_principle_axis, FloorAxisAlignment, align_principle_axis::FloorAxisAlignment, align_principle_axis::AxisAlignment)

namespace align_principle_axis
{
  FloorAxisAlignment::FloorAxisAlignment ()
  {
    // TODO Auto-generated constructor stub

  }

  FloorAxisAlignment::~FloorAxisAlignment ()
  {
    // TODO Auto-generated destructor stub
  }

  void FloorAxisAlignment::alignCloudPrincipleAxis (const PointCloudConstPtr input_cloud_ptr,
      const Eigen::Matrix4f& inital_guess, const PointCloudPtr output_cloud_ptr,
      Eigen::Matrix4f& transform_output)
  {
    PointCloudPtr rotated_cloud (new PointCloud);
    transformPointCloud (*input_cloud_ptr, *rotated_cloud, inital_guess);

    //assume the model is approx correct, i.e. z is height
    // find the largest plane perpendicular to z axis and use this to align cloud
    pcl17::ModelCoefficients::Ptr coefficients (new pcl17::ModelCoefficients);
    pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices);
    pcl17::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl17::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType (pcl17::SAC_RANSAC);
    seg.setDistanceThreshold (0.05);

    seg.setAxis (Eigen::Vector3f (0, 0, 1));
    seg.setEpsAngle (10.0 * (M_PI / 180));//radians
    seg.setInputCloud (rotated_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL17_ERROR("Could not estimate a planar model for the given dataset.");
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1]
    << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

    // calculate angle and axis from dot product
    double rotate = acos (coefficients->values[2]);
    Eigen::Vector3f axis_to_rotate_about (-coefficients->values[1], coefficients->values[0], 0.0);
    axis_to_rotate_about.normalize ();

    //convert axis and angle to rotation matrix and apply to pointcloud
    Eigen::AngleAxis<float> angle_axis (rotate, axis_to_rotate_about);
    transform_output = Eigen::Matrix4f::Identity (4, 4);
    transform_output.block<3, 3> (0, 0) = angle_axis.toRotationMatrix ().inverse ();
    transformPointCloud (*rotated_cloud, *output_cloud_ptr, transform_output);

//  pcl17::ExtractIndices<PointType> eifilter;
//   eifilter.setInputCloud (rotated_cloud);
//   eifilter.setIndices (inliers);
//   eifilter.filter (*output_cloud_ptr);
  }
}
