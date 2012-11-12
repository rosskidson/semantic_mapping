/*
 * box_filter.cpp
 *
 *  Created on: Nov 11, 2012
 *      Author: kidson
 */

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Core>


bool filterCloud ()
{
  //convert units
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  float x,y,z;
  x=y=z=0.0;
  Eigen::Vector4f max_point (x,y,z,1);
  Eigen::Vector4f min_point (x,z,y,1);

  // apply filter
  pcl::CropBox<pcl::PointXYZ> box_filter;
  box_filter.setInputCloud(input_cloud);
  box_filter.setMin(min_point);
  box_filter.setMax(max_point);
  box_filter.filter(*output_cloud);

  return true;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "box_filter");
  ros::NodeHandle nh ("~");

  return 0;
}
