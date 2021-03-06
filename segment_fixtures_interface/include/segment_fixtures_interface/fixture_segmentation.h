/*
 * fixture_segmentation.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: ross kidson
 */

#ifndef FIXTURE_SEGMENTATION_H_
#define FIXTURE_SEGMENTATION_H_

#include <pcl_typedefs/pcl_typedefs.h>

#include <pcl17/ModelCoefficients.h>

namespace segment_fixtures_interface
{
  class FixtureSegmentation
  {
    public:
      FixtureSegmentation (){};
      virtual ~FixtureSegmentation (){};

      virtual void segmentFixtures(const PointCloudConstPtr model, std::vector<pcl17::PointIndicesConstPtr>& fixture_indices_ptrs) = 0;

      virtual void setPlanes(std::vector<pcl17::PointIndicesConstPtr>& plane_indices_ptrs,
            std::vector<pcl17::ModelCoefficients::ConstPtr>& plane_coeffs){};

  };
}

#endif /* FIXTURE_SEGMENTATION_H_ */

