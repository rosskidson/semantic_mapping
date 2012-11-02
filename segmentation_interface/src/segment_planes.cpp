/*
 * segment_planes.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: kidson
 */

#include "segmentation_interface/segment_planes.h"
#include <ros/console.h>

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
