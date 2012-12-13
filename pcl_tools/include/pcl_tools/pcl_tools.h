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

  void filterCloud (const PointCloudConstPtr input_cloud_ptr, const Eigen::Vector4f& min_point,
      const Eigen::Vector4f& max_point, const PointCloudPtr output_cloud_ptr);

  void moveModelToOrigin (const PointCloudConstPtr cloud_input_ptr,
      const PointCloudPtr cloud_output_ptr,
      Eigen::Matrix4f& transform_output);

  pcl17::PointIndicesPtr getIndicesFromPointCloud(const PointCloudConstPtr& input_cloud_ptr,
                                                  const pcl17::PointIndicesConstPtr& input_indices_ptr,
                                                  const PointCloudConstPtr& search_cloud_ptr);

  pcl17::PointIndicesPtr getIndicesFromPointCloud(const PointCloudConstPtr& source_cloud,
                                                  const PointCloudConstPtr& search_cloud_ptr);

}
