/*
 * main.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: kidson
 */

/* ** OLD STYLE:  (DIRECT DEPENDENCY)

#include <ros/ros.h>
#include <segment_planes/segment_planes.h>
#include <segment_fixtures/segment_fixtures.h>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test");
  ros::NodeHandle nh("~");

  segment_fixtures::SegmentFixtures seg_obj;
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

***/

// New style (only plugin loader required)

#include <pluginlib/class_loader.h>
#include <segmentation_interface/segmentation.h>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<segmentation_interface::Segmentation> seg_loader("segmentation_interface", "segmentation_interface::Segmentation");

  segmentation_interface::Segmentation* planes = NULL;
  segmentation_interface::Segmentation* fixtures = NULL;

  try
  {
    std::vector<PointCloudConstPtr> segmented_clouds;

    planes = seg_loader.createClassInstance("segment_planes/SegmentPlanes");
    planes->segment(segmented_clouds);

    fixtures = seg_loader.createClassInstance("segment_fixtures/SegmentFixtures");
    fixtures->segment(segmented_clouds);

 //   ROS_INFO("Triangle area: %.2f", triangle->area());
 //   ROS_INFO("Square area: %.2f", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}

