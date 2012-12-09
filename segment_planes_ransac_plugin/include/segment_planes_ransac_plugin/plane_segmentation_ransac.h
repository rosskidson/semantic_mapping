/*
 * PlaneSegmentationRANSAC.h
 *
 *  Created on: Nov 26, 2012
 *      Author: kidson
 */

#ifndef PLANESEGMENTATIONRANSAC_H_
#define PLANESEGMENTATIONRANSAC_H_

#include <segment_planes_interface/plane_segmentation.h>

namespace segment_planes_ransac_plugin
{
  class PlaneSegmentationRANSAC : public segment_planes_interface::PlaneSegmentation
  {
    public:
      PlaneSegmentationRANSAC ();
      virtual ~PlaneSegmentationRANSAC ();

      virtual void segmentPlanes(const PointCloudConstPtr model,
          std::vector<PointCloudConstPtr>& plane_clouds,
          std::vector<pcl17::ModelCoefficients::ConstPtr>& plane_coeffs);

      virtual void setNormals(const PointCloudNormalsConstPtr normals);

  };
}

#endif /* PLANESEGMENTATIONRANSAC_H_ */
