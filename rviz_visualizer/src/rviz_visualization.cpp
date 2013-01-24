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

#include <iostream>

RVizVisualization::RVizVisualization ()
{

}

RVizVisualization::~RVizVisualization ()
{

}

int RVizVisualization::addCloudToVisualizer (PointCloudConstPtr cloud_ptr, double red, double green,double blue)
{

}

void RVizVisualization::addNormalsToVisualizer (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr)
{

}

void RVizVisualization::visualizeImage(const sensor_msgs::Image& image_msg)
{

}

void RVizVisualization::removeAllClouds()
{

}
