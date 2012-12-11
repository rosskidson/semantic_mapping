/*
 * FixtureSegmentationFromPlanes.cpp
 *
 *  Created on: Dec 11, 2012
 *      Author: ross kidson
 */

#include "segment_fixtures_from_planes_plugin/fixture_segmentation_from_planes.h"

//#include <pcl17/search/search.h>
//#include <pcl17/search/kdtree.h>
//#include <pcl17/segmentation/region_growing.h>

//#include <pcl17/filters/extract_indices.h>

#include <ros/console.h>

//pluginlib
#include <pluginlib/class_list_macros.h>

//Declare the plugin
//param_1: The namespace in which the  plugin will live
//param_2: The name we wish to give to the plugin
// loader.createClassInstance("param_1/param_2");
//param_3: The fully-qualified type of the plugin class
//param_4: The fully-qualified type of the base class
PLUGINLIB_DECLARE_CLASS(segment_fixtures_from_planes_plugin, FixtureSegmentationFromPlanes, segment_fixtures_from_planes_plugin::FixtureSegmentationFromPlanes, segment_fixtures_interface::FixtureSegmentation)

namespace segment_fixtures_from_planes_plugin
{

  FixtureSegmentationFromPlanes::FixtureSegmentationFromPlanes ():
    //normals_ptr_(new pcl17::PointCloud<pcl17::Normal>),
    nh_("~/fixture_segmentation_from_planes")
    //reconfig_srv_(nh_)
  {
    //reconfig_callback_ = boost::bind (&FixtureSegmentationFromPlanes::reconfigCallback, this, _1, _2);
    //reconfig_srv_.setCallback (reconfig_callback_);

  }

  FixtureSegmentationFromPlanes::~FixtureSegmentationFromPlanes ()
  {
    // TODO Auto-generated destructor stub
  }

//  void FixtureSegmentationFromPlanes::reconfigCallback (segment_planes_region_grow_plugin::PlaneSegmentationConfig &config,
//      uint32_t level)
//  {
//    //  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
//    //            config.int_param, config.double_param,
//    //            config.str_param.c_str(),
//    //            config.bool_param?"True":"False",
//    //            config.size);


//  }

  void FixtureSegmentationFromPlanes::setPlanes(std::vector<PointCloudConstPtr>& plane_clouds,
                                                std::vector<pcl17::ModelCoefficients::ConstPtr>& plane_coeffs)
  {
    //pcl17::copyPointCloud(*normals, *normals_ptr_);
  }

  void FixtureSegmentationFromPlanes::segmentFixtures(const PointCloudConstPtr model)
  {
    ROS_INFO("segment fixtures from planes");

  }

}
