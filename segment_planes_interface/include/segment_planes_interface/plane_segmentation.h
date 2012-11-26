/*
 * plane_segmentation.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: kidson
 */

#ifndef PLANE_SEGMENTATION_H_
#define PLANE_SEGMENTATION_H_

#include <pcl_typedefs/pcl_typedefs.h>

//#include <pcl/point_types.h>
//typedef pcl::PointXYZ PointType;
//typedef pcl::PointNormal PointNormal;
//
//typedef pcl::PointCloud<PointType> PointCloud;
//typedef PointCloud::Ptr PointCloudPtr;
//typedef PointCloud::ConstPtr PointCloudConstPtr;
//
//typedef pcl::PointCloud<PointNormal> PointCloudNormals;
//typedef PointCloudNormals::Ptr PointCloudNormalsPtr;
//typedef PointCloudNormals::ConstPtr PointCloudNormalsConstPtr;

#include <pcl/ModelCoefficients.h>

namespace segment_planes_interface
{
  class PlaneSegmentation
  {
    public:
      PlaneSegmentation (){};
      virtual ~PlaneSegmentation (){};

      virtual void segmentPlanes(const PointCloudConstPtr model,
          const std::vector<PointCloudConstPtr>& plane_clouds,
          const std::vector<pcl::ModelCoefficients::ConstPtr>& plane_coeffs)=0;

  };
}

#endif /* PLANE_SEGMENTATION_H_ */
