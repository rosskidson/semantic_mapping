/*
 * fixture_segmentation.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: ross kidson
 */

#ifndef FIXTURE_SEGMENTATION_H_
#define FIXTURE_SEGMENTATION_H_

#include <pcl_typedefs/pcl_typedefs.h>

#include <pcl/ModelCoefficients.h>

namespace segment_fixtures_interface
{
  class FixtureSegmentation
  {
    public:
      FixtureSegmentation (){};
      virtual ~FixtureSegmentation (){};

      virtual void segmentFixtures(const PointCloudConstPtr model, std::vector<pcl::PointIndicesConstPtr>& fixture_indices_ptrs) = 0;

      virtual void setPlanes(std::vector<pcl::PointIndicesConstPtr>& plane_indices_ptrs,
            std::vector<pcl::ModelCoefficients::ConstPtr>& plane_coeffs){};

  };
}

#endif /* FIXTURE_SEGMENTATION_H_ */

