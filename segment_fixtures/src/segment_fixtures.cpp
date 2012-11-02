/*
 * segment_fixtures.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: kidson
 */

#include <ros/console.h>
#include "segment_fixtures/segment_fixtures.h"
#include <pluginlib/class_list_macros.h>

//Declare the plane segmentation as a segmentation class
PLUGINLIB_DECLARE_CLASS(segment_fixtures, SegmentFixtures, segment_fixtures::SegmentFixtures, segmentation_interface::Segmentation)

namespace segment_fixtures
{

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
}
