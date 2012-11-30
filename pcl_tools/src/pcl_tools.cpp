/*
 * pcl_tools.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: ross kidson
 */

#include "pcl_tools/pcl_tools.h"

#include <pcl_typedefs/pcl_typedefs.h>

//normal estimation
#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/features/normal_3d_omp.h>
//voxel grid filter
#include <pcl17/filters/voxel_grid.h>
// tranform cloud
#include <pcl17/common/transforms.h>
// conversions
#include "cv_bridge/cv_bridge.h"
#include "pcl17/ros/conversions.h"

namespace pcl_tools
{
  void calculateNormals (PointCloudConstPtr input_cloud_ptr, PointCloudNormalsPtr normals_ptr)
  {
    pcl17::NormalEstimation<PointType, PointNormal> normal_est;
    normal_est.setInputCloud (input_cloud_ptr);
    pcl17::search::KdTree<PointType>::Ptr tree (new pcl17::search::KdTree<PointType> ());
    normal_est.setSearchMethod (tree);
    normal_est.setRadiusSearch (0.05);
    normal_est.compute (*normals_ptr);
    //pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
  }

  void calculateNormalsOMP (PointCloudConstPtr input_cloud_ptr, PointCloudNormalsPtr normals_ptr)
  {
    pcl17::NormalEstimationOMP<PointType, PointNormal> normal_est;
    normal_est.setInputCloud (input_cloud_ptr);
    pcl17::search::KdTree<PointType>::Ptr tree (new pcl17::search::KdTree<PointType> ());
    normal_est.setSearchMethod (tree);
    normal_est.setRadiusSearch (0.05);
    normal_est.compute (*normals_ptr);
    //pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
  }

  void transformPointCloud(PointCloudConstPtr input_cloud_ptr, PointCloudPtr output_cloud_ptr, const Eigen::Matrix4f& transform )
  {
   transformPointCloud (*input_cloud_ptr, *output_cloud_ptr, transform);
  }

  // sensor_msg::Image -> cv::Mat
  cv::Mat convertSensorMsgToCV (const sensor_msgs::Image& ros_image)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy (ros_image, ros_image.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR ("cv_bridge exception: %s", e.what ());
    }
    return cv_ptr->image;
  }

  // Pointcloud_2 -> pcl pointcloud
  PointCloudPtr convertSensorMsgPointCloudToPCL (sensor_msgs::PointCloud2& ros_pointcloud)
  {
    PointCloudPtr pointcloud_ptr (new PointCloud);
    pcl17::fromROSMsg (ros_pointcloud, *pointcloud_ptr);
    return pointcloud_ptr;
  }

  PointCloudPtr downsampleCloud(const PointCloudConstPtr input_cloud_ptr, const float leaf_size)
  {
    pcl17::VoxelGrid<PointType> sor;
    PointCloudPtr output_cloud_ptr (new PointCloud);
    sor.setInputCloud (input_cloud_ptr);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter (*output_cloud_ptr);
    return output_cloud_ptr;
  }

}
