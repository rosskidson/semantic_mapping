/*
 * segment_planes.h
 *
 *  Created on: Nov 2, 2012
 *      Author: kidson
 */

#ifndef SEGMENT_PLANES_H_
#define SEGMENT_PLANES_H_

#include <segmentation_interface/segmentation.h>

namespace segment_planes
{
  class SegmentPlanes : public segmentation_interface::Segmentation
  {
    public:
      SegmentPlanes ();
      virtual ~SegmentPlanes ();

      void segment (std::vector<PointCloudConstPtr> segmented_clouds);
  };
};
#endif /* SEGMENT_PLANES_H_ */
