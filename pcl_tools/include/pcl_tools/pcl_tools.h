/*
 * pcl_tools.h
 *
 *  Created on: Nov 26, 2012
 *      Author: ross kidson
 */

#include "pcl_typedefs/pcl_typedefs.h"
#include <cv_bridge/cv_bridge.h>

namespace pcl_tools
{

  void calculateNormals (PointCloudConstPtr input_cloud_ptr, PointCloudNormalsPtr normals_ptr);

  void calculateNormalsOMP (PointCloudConstPtr input_cloud_ptr, PointCloudNormalsPtr normals_ptr);

  void transformPointCloud(PointCloudConstPtr input_cloud_ptr, PointCloudPtr output_cloud_ptr, const Eigen::Matrix4f &transform );

  cv::Mat convertSensorMsgToCV (const sensor_msgs::Image& ros_image);

  PointCloudPtr convertSensorMsgPointCloudToPCL (sensor_msgs::PointCloud2& ros_pointcloud);

  PointCloudPtr downsampleCloud(const PointCloudConstPtr input_cloud_ptr, const float leaf_size=0.01);

}
