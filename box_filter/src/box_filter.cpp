/*
 * box_filter.cpp
 *
 *  Created on: Nov 11, 2012
 *      Author: kidson
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/crop_box.h>

//#include "pcl_tools/boxFilter.h"

void filterCloud()
{
  pcl::CropBox<pcl::PointXYZ> box_filter;
//  box_filter.setInputCloud()

}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "box_filter_service");
  ros::NodeHandle nh("~");
//    ros::ServiceServer service = nh.advertiseService("box_filter_service", box_filter);
  box_filter();

  ros::spin();

  return 0;
}
