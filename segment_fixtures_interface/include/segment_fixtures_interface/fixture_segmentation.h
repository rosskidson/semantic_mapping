/*
 * fixture_segmentation.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: ross kidson
 */

#ifndef FIXTURE_SEGMENTATION_H_
#define FIXTURE_SEGMENTATION_H_

#include <pcl_typedefs/pcl_typedefs.h>

namespace segment_fixtures_interface
{
  class FixtureSegmentation
  {
    public:
      FixtureSegmentation (){};
      virtual ~FixtureSegmentation (){};

      virtual void segmentFixtures(const PointCloudConstPtr model,
          const std::vector<PointCloudConstPtr>& plane_cloud_ptrs,
          const std::vector<pcl::ModelCoefficients> plane_coefficients,
          std::vector<PointCloudConstPtr>& plane_clouds)=0;
  };
}

#endif /* FIXTURE_SEGMENTATION_H_ */

