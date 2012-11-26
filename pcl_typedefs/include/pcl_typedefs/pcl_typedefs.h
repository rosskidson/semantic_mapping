#ifndef PCL_TYPEDEFS_H_
#define PCL_TYPEDEFS_H_

#include <pcl17/point_types.h>

typedef pcl17::PointXYZ PointType;
typedef pcl17::PointNormal PointNormal;

typedef pcl17::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

typedef pcl17::PointCloud<PointNormal> PointCloudNormals;
typedef PointCloudNormals::Ptr PointCloudNormalsPtr;
typedef PointCloudNormals::ConstPtr PointCloudNormalsConstPtr;

#endif
