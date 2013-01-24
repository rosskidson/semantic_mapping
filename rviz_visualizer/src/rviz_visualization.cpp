/*
 * rviz_visualization.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: Ross Kidson
 */

#include "rviz_visualizer/rviz_visualization.h"

// ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/ros.h>
#include <ros/console.h>

//pcl
#include <pcl17/point_types.h>
#include <pcl17/ros/conversions.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/filters/extract_indices.h>

//visualization msgs
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

//interactive markers
#include <interactive_markers/interactive_marker_server.h>


#include <iostream>

RVizVisualization::RVizVisualization ():
  interactive_marker_server_objects_("semantic_mapping")
{

}

RVizVisualization::~RVizVisualization ()
{

}

/**
 *
 * @param[in] cloud_ptr  Pointer to pointcloud
 * @param[in] red Specify colour for the cloud to visualize
 * @param[in] green Specify colour for the cloud to visualize
 * @param[in] blue Specify colour for the cloud to visualize
 * @return id of the cloud in the visualization obj.
 */
int RVizVisualization::addCloudToVisualizer (PointCloudConstPtr cloud_ptr, double red, double green,double blue)
{
  // create marker object
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::POINTS;
  geometry_msgs::Point p;
  for (size_t j = 0; j < cloud_ptr->points.size(); j++)
  {
    p.x = cloud_ptr->points.at(j).x;
    p.y = cloud_ptr->points.at(j).y;
    p.z = cloud_ptr->points.at(j).z;
    marker.points.push_back(p);
  }
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration(0);
  marker.scale.x = 0.003;
  marker.scale.y = 0.003;
  marker.scale.z = 0.003;

  // create control object
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.markers.push_back(marker);

  // create point cloud marker
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.name = "cloud";
  int_marker.description = "PointCloud";
  int_marker.controls.push_back(control);

  interactive_marker_server_objects_.insert(int_marker);
  //interactive_marker_server_objects_->setCallback(int_marker.name, boost::bind(&InteractiveMarkerPublisher::processObjectFeedback, this, _1));
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

void RVizVisualization::spinOnce()
{

}
