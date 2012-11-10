/*
 * segment_planes.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: kidson
 */

#include <ros/console.h>
#include "segment_planes/segment_planes.h"
#include <pluginlib/class_list_macros.h>

//Declare the plane segmentation as a segmentation class
PLUGINLIB_DECLARE_CLASS(segment_planes, SegmentPlanes, segment_planes::SegmentPlanes, segmentation_interface::Segmentation)
namespace segment_planes
{
  SegmentPlanes::SegmentPlanes ()
  {
    // TODO Auto-generated constructor stub

  }

  SegmentPlanes::~SegmentPlanes ()
  {
    // TODO Auto-generated destructor stub
  }

  void SegmentPlanes::segment (std::vector<PointCloudConstPtr> segmented_clouds)
  {
    ROS_INFO_STREAM("segment planes");
  }
};
