/*
 * rviz_visualization.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: Ross Kidson
 */

#include "rviz_visualizer/rviz_visualization.h"

// ros
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <ros/console.h>

//pcl
#include <pcl17/point_types.h>
#include <pcl17/ros/conversions.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/filters/extract_indices.h>

// opencv -> ROS -> opencv
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

// opencv
//#include <opencv2/highgui/highgui.hpp>

#include <iostream>

RVizVisualization::RVizVisualization ():
  vox_grid_size_(0.0)
{

}

RVizVisualization::~RVizVisualization ()
{
  // TODO Auto-generated destructor stub
}

void RVizVisualization::visualizeCloud (const sensor_msgs::PointCloud2& pointcloud_msg)
{
  PointCloudPtr cloud_ptr (new PointCloud);
  pcl17::fromROSMsg(pointcloud_msg,*cloud_ptr);
  visualizeCloud(cloud_ptr);
}

void RVizVisualization::visualizeCloud (PointCloudConstPtr cloud_ptr)
{
  std::vector<PointCloudConstPtr> cloud_ptr_vec;
  cloud_ptr_vec.push_back(cloud_ptr);
  visualizeCloud(cloud_ptr_vec);
}

void RVizVisualization::visualizeCloud (std::vector<PointCloudConstPtr>& cloud_ptr_vec)
{

}

void RVizVisualization::visualizeCloud (PointCloudConstPtr cloud_ptr, pcl17::PointIndicesConstPtr& cloud_indices_ptr)
{
  pcl17::ExtractIndices<PointType> extractor;
  PointCloudPtr output_cloud_ptr (new PointCloud);
  extractor.setIndices(cloud_indices_ptr);
  extractor.setInputCloud(cloud_ptr);
  extractor.filter(*output_cloud_ptr);
  visualizeCloud(output_cloud_ptr);
}

void RVizVisualization::visualizeCloud (PointCloudConstPtr cloud_ptr, std::vector<pcl17::PointIndicesConstPtr>& cloud_indices_ptrs)
{
  std::vector<PointCloudConstPtr> clouds_to_visualize_ptrs;
  for(std::vector<pcl17::PointIndicesConstPtr>::const_iterator itr=cloud_indices_ptrs.begin(); itr!=cloud_indices_ptrs.end(); itr++)
  {
    pcl17::ExtractIndices<PointType> extractor;
    PointCloudPtr output_cloud_ptr (new PointCloud);
    extractor.setIndices(*itr);
    extractor.setInputCloud(cloud_ptr);
    extractor.filter(*output_cloud_ptr);
    clouds_to_visualize_ptrs.push_back(output_cloud_ptr);
  }
  visualizeCloud(clouds_to_visualize_ptrs);
}

void RVizVisualization::visualizeCloudNormals (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr)
{

}

void RVizVisualization::visualizeImage(const sensor_msgs::Image& image_msg)
{

}

PointCloudConstPtr RVizVisualization::downsampleCloud (PointCloudConstPtr input)
{
  const double voxel_size = vox_grid_size_;
  PointCloudPtr cloud_filtered (new PointCloud);
  pcl17::VoxelGrid<PointType> downsampler;
  downsampler.setInputCloud (input);
  downsampler.setLeafSize (voxel_size, voxel_size, voxel_size);
  downsampler.filter (*cloud_filtered);
  return cloud_filtered;
}