/*
 * plane_segmentation.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: kidson
 */

#ifndef PLANE_SEGMENTATION_H_
#define PLANE_SEGMENTATION_H_

#include <pcl_typedefs/pcl_typedefs.h>

#include <pcl/ModelCoefficients.h>

namespace segment_planes_interface
{
  class PlaneSegmentation
  {
    public:
      PlaneSegmentation (){};
      virtual ~PlaneSegmentation (){};

      virtual void segmentPlanes(const PointCloudConstPtr model,
          std::vector<pcl::PointIndicesConstPtr>& plane_indices_ptrs,
          std::vector<pcl::ModelCoefficients::ConstPtr>& plane_coeffs)=0;

      virtual void setNormals(const PointCloudNormalsConstPtr normals) = 0;

  };
}

#endif /* PLANE_SEGMENTATION_H_ */
