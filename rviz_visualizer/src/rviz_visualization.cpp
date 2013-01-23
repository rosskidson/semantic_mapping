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

RVizVisualization::RVizVisualization ()
{

}

RVizVisualization::~RVizVisualization ()
{
  // TODO Auto-generated destructor stub
}

void Visualization::visualizeClouds (std::vector<PointCloudConstPtr>& cloud_ptr_vec)
{

  for(std::vector<PointCloudConstPtr>::iterator itr=cloud_ptr_vec.begin(); itr != cloud_ptr_vec.end(); itr++)
  {
    int cloud_num = itr - cloud_ptr_vec.begin();
    std::stringstream cloud_name;
    cloud_name << "cloud "<< itr - cloud_ptr_vec.begin();

    // downsample cloud if needed
    PointCloudConstPtr downsampled_ptr;
    if(vox_grid_size_ > 0.0)
      downsampled_ptr = downsampleCloud(*itr);
    else
      downsampled_ptr = *itr;

    // different colours for different clouds
    pcl17::visualization::PointCloudColorHandlerCustom<PointType>
      single_color(downsampled_ptr, 50 + rand() % 205, 50 + rand() % 205, 50 + rand() % 205);

    //add the cloud

  }

}

void RVizVisualization::visualizeCloudNormals (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr)
{

}

void RVizVisualization::visualizeImage(const sensor_msgs::Image& image_msg)
{

}
