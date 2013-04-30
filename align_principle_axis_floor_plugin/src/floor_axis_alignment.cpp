/*
 * floor_axis_alignment.cpp
 *
 *  Created on: 20/11/2012
 *      Author: ross
 */

#include "align_principle_axis_floor_plugin/floor_axis_alignment.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include <pluginlib/class_list_macros.h>

#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/align_principle_axis_floor_plugin/FloorAxisAlignmentConfig.h"

//Declare the plugin
//param_1: The namespace in which the  plugin will live
//param_2: The name we wish to give to the plugin
// loader.createClassInstance("param_1/param_2");
//param_3: The fully-qualified type of the plugin class
//param_4: The fully-qualified type of the base class
PLUGINLIB_DECLARE_CLASS(align_principle_axis_floor_plugin, FloorAxisAlignment, align_principle_axis_floor_plugin::FloorAxisAlignment, align_principle_axis_interface::AxisAlignment)

namespace align_principle_axis_floor_plugin
{
  FloorAxisAlignment::FloorAxisAlignment ():
    nh_("~/floor_axis_alignment"),
    reconfig_srv_(nh_)
  {
    reconfig_srv_.setCallback (boost::bind (&FloorAxisAlignment::reconfigCallback, this, _1, _2));
  }

  FloorAxisAlignment::~FloorAxisAlignment ()
  {
    // TODO Auto-generated destructor stub
  }

  void FloorAxisAlignment::reconfigCallback (align_principle_axis_floor_plugin::FloorAxisAlignmentConfig &config, uint32_t level)
  {
    //  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
    //            config.int_param, config.double_param,
    //            config.str_param.c_str(),
    //            config.bool_param?"True":"False",
    //            config.size);

   if(config.axis_to_rotate_about == 0)
       axis_ = Eigen::Vector3f::UnitX();
   else if(config.axis_to_rotate_about == 1)
     axis_ = Eigen::Vector3f::UnitY();
   else if(config.axis_to_rotate_about == 2)
     axis_ = Eigen::Vector3f::UnitZ();

   angle_ = config.angle * (M_PI/180.0);

   ransac_threshold_ = config.ransac_distance_threshold;
   max_iterations_ = config.max_iterations;

  }

  void FloorAxisAlignment::alignCloudPrincipleAxis (const PointCloudConstPtr input_cloud_ptr,
      const PointCloudPtr output_cloud_ptr,
      Eigen::Matrix4f& transform_output)
  {
    Eigen::Matrix4f inital_guess;
    inital_guess.block<3,3>(0,0) = Eigen::AngleAxisf(angle_, axis_).toRotationMatrix();
//    inital_guess = Eigen::Matrix4f::Zero(4,4);
//    inital_guess(0,0) = 1.0;
//    inital_guess(1,2) = 1.0;
//    inital_guess(2,1) = -1.0;
//    inital_guess(3,3) = 1.0;
    PointCloudPtr rotated_cloud_ptr (new PointCloud);
    transformPointCloud (*input_cloud_ptr, *rotated_cloud_ptr, inital_guess);
    //transformPointCloud (*input_cloud_ptr, *output_cloud_ptr, inital_guess);

    //assume the model is approx correct, i.e. z is height
    // find the largest plane perpendicular to z axis and use this to align cloud
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (ransac_threshold_);
    seg.setMaxIterations(max_iterations_);
    //seg.setProbability(0.1);

    seg.setAxis (Eigen::Vector3f (0, 0, 1));
    seg.setEpsAngle (30.0 * (M_PI / 180));//radians
    seg.setInputCloud (rotated_cloud_ptr);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }

      // I had some issues with pcl on a different computer to where I normally dev.  This is a workaround to be removed
    //  coefficients->values.push_back(-0.0370391);
    //  coefficients->values.push_back(0.0777064);
    //  coefficients->values.push_back(0.996288);
    //  coefficients->values.push_back(2.63374);

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

    // apply translation to bring the floor to z = 0
    transform_output(2, 3) = coefficients->values[3];
    transformPointCloud (*rotated_cloud_ptr, *output_cloud_ptr, transform_output);

//  pcl::ExtractIndices<PointType> eifilter;
//   eifilter.setInputCloud (rotated_cloud_ptr);
//   eifilter.setIndices (inliers);
//   eifilter.filter (*output_cloud_ptr);
  }
}
