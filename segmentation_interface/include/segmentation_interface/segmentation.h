/*
 * Segmentation.h
 *
 *  Created on: Nov 2, 2012
 *      Author: kidson
 */

#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// pcl typedefs
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormals;
typedef PointCloudNormals::Ptr PointCloudNormalsPtr;
typedef PointCloudNormals::ConstPtr PointCloudNormalsConstPtr;

namespace segmentation_interface
{
  class Segmentation
  {
    public:
      Segmentation (){};
      virtual ~Segmentation (){};

      void virtual segment (std::vector<PointCloudConstPtr> segmented_clouds)=0;

      //void setInputCloud (const PointCloudConstPtr input);

    private:
      PointCloudConstPtr input_cloud_;
  };
}
#endif /* SEGMENTATION_H_ */
