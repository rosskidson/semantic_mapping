/*
 * FixtureSegmentationFromPlanes.h
 *
 *  Created on: Dec 11, 2012
 *      Author: kidson
 */

#ifndef FIXTURESEGMENTATIONFROMPLANES_H_
#define FIXTURESEGMENTATIONFROMPLANES_H_

#include "ros/ros.h"
#include <pcl_typedefs/pcl_typedefs.h>
#include <segment_fixtures_interface/fixture_segmentation.h>

//#include <dynamic_reconfigure/server.h>
//#include "../../cfg/cpp/segment_planes_region_grow_plugin/PlaneSegmentationConfig.h"

namespace segment_fixtures_from_planes_plugin
{
  class FixtureSegmentationFromPlanes : public segment_fixtures_interface::FixtureSegmentation
  {
    public:
      FixtureSegmentationFromPlanes();
      virtual ~FixtureSegmentationFromPlanes();

      virtual void segmentFixtures(const PointCloudConstPtr model);

      virtual void setPlanes(std::vector<PointCloudConstPtr>& plane_clouds,
            std::vector<pcl17::ModelCoefficients::ConstPtr>& plane_coeffs);

    private:

     // void reconfigCallback (segment_fixtures_from_planes_plugin::FixtureSegmentationFromPlanesConfig &config,
     //     uint32_t level);

      std::vector<PointCloudConstPtr> plane_clouds;
      std::vector<pcl17::ModelCoefficients::ConstPtr> plane_coeffs;

      ros::NodeHandle nh_;
      //dynamic_reconfigure::Server<segment_fixtures_from_planes_plugin::FixtureSegmentationConfig> reconfig_srv_;
      //dynamic_reconfigure::Server<segment_fixtures_from_planes_plugin::FixtureSegmentationConfig>::CallbackType
      //    reconfig_callback_;

  };
}

#endif /* FIXTURESEGMENTATIONFROMPLANES_H_ */
