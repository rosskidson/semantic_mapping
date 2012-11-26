/*
 * PlaneSegmentationRANSAC.h
 *
 *  Created on: Nov 26, 2012
 *      Author: kidson
 */

#ifndef PLANESEGMENTATIONREGIONGROW_H_
#define PLANESEGMENTATIONREGIONGROW_H_

#include <segment_planes_interface/plane_segmentation.h>

namespace segment_planes_region_grow_plugin
{
  class PlaneSegmentationRegionGrow : public segment_planes_interface::PlaneSegmentation
  {
    public:
      PlaneSegmentationRegionGrow();
      virtual ~PlaneSegmentationRegionGrow();

      virtual void segmentPlanes(const PointCloudConstPtr model,
          const std::vector<PointCloudConstPtr>& plane_clouds,
          const std::vector<pcl::ModelCoefficients::ConstPtr>& plane_coeffs);

  };
}

#endif /* PLANESEGMENTATIONREGIONGROW_H_ */
