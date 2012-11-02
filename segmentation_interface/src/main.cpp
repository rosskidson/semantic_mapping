/*
 * main.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: kidson
 */

#include <ros/ros.h>
#include "segmentation_interface/segment_fixtures.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test");
  ros::NodeHandle nh("~");

  SegmentFixtures seg_obj;
  ros::Rate loop_rate (100);
  std::vector<PointCloudConstPtr> segmented_clouds;
  seg_obj.segment(segmented_clouds);
//  while (ros::ok ())
//  {
//    ros::spinOnce ();
//    loop_rate.sleep ();
//  }
  return 0;

}

