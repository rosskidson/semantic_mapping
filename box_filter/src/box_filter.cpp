/*
 * box_filter.cpp
 *
 *  Created on: Nov 11, 2012
 *      Author: kidson
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Core>

#include "box_filter/boxFilter.h"

bool filterCloud (box_filter::boxFilter::Request& req, box_filter::boxFilter::Response& res)
{
  //convert units
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(req.input_cloud, *input_cloud);
  Eigen::Vector4f max_point (req.input_max_point.x, req.input_max_point.y, req.input_max_point.z,1);
  Eigen::Vector4f min_point (req.input_min_point.x, req.input_min_point.y, req.input_min_point.z,1);

  // apply filter
  pcl::CropBox<pcl::PointXYZ> box_filter;
  box_filter.setInputCloud(input_cloud);
  box_filter.setMin(min_point);
  box_filter.setMax(max_point);
  box_filter.filter(*output_cloud);

  //convert back
  pcl::toROSMsg(*output_cloud, res.output);
  return true;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "box_filter");
  ros::NodeHandle nh ("~");
  ros::ServiceServer service = nh.advertiseService("box_filter", filterCloud);
  ROS_INFO("Box filter service up and running");
  ros::spin ();

  return 0;
}
