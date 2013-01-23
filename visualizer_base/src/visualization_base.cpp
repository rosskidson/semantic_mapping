/*
 * visualization_base.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include "visualizer_base/visualization_base.h"

// ros
#include <sensor_msgs/PointCloud2.h>

//pcl
#include <pcl17/point_types.h>
#include <pcl17/ros/conversions.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/filters/extract_indices.h>


#include <iostream>

VisualizationBase::VisualizationBase ():
  vox_grid_size_(0.0)
{

}

VisualizationBase::~VisualizationBase ()
{
  // TODO Auto-generated destructor stub
}

int VisualizationBase::visualizeCloud (const sensor_msgs::PointCloud2& pointcloud_msg)
{
  PointCloudPtr cloud_ptr (new PointCloud);
  pcl17::fromROSMsg(pointcloud_msg,*cloud_ptr);

  return visualizeCloud(cloud_ptr);
}

int VisualizationBase::visualizeCloud (PointCloudConstPtr cloud_ptr, pcl17::PointIndicesConstPtr& cloud_indices_ptr)
{
  pcl17::ExtractIndices<PointType> extractor;
  PointCloudPtr output_cloud_ptr (new PointCloud);
  extractor.setIndices(cloud_indices_ptr);
  extractor.setInputCloud(cloud_ptr);
  extractor.filter(*output_cloud_ptr);
  return visualizeCloud(output_cloud_ptr);
}

int VisualizationBase::visualizeCloud (PointCloudConstPtr cloud_ptr)
{
  removeAllClouds();
  // downsample cloud if needed
  PointCloudConstPtr downsampled_ptr;
  if(vox_grid_size_ > 0.0)
    downsampled_ptr = downsampleCloud(cloud_ptr);
  else
    downsampled_ptr = cloud_ptr;

  return addCloudToVisualizer(downsampled_ptr,  50 + rand() % 205, 50 + rand() % 205, 50 + rand() % 205);
}

void VisualizationBase::visualizeCloudNormals (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr)
{
  removeAllClouds();
  visualizeCloudNormals(cloud_ptr,cloud_normals_ptr);
}

PointCloudConstPtr VisualizationBase::downsampleCloud (PointCloudConstPtr input)
{
  const double voxel_size = vox_grid_size_;
  PointCloudPtr cloud_filtered (new PointCloud);
  pcl17::VoxelGrid<PointType> downsampler;
  downsampler.setInputCloud (input);
  downsampler.setLeafSize (voxel_size, voxel_size, voxel_size);
  downsampler.filter (*cloud_filtered);
  return cloud_filtered;
}
