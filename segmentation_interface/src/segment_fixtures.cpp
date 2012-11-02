/*
 * segment_fixtures.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: kidson
 */

#include "segmentation_interface/segment_fixtures.h"
#include <ros/console.h>

SegmentFixtures::SegmentFixtures ()
{
  // TODO Auto-generated constructor stub

}

SegmentFixtures::~SegmentFixtures ()
{
  // TODO Auto-generated destructor stub
}

void SegmentFixtures::segment (std::vector<PointCloudConstPtr> segmented_clouds)
{
  ROS_INFO_STREAM("segment fixtures");
}
