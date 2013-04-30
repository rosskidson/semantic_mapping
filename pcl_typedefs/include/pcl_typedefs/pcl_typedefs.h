#ifndef PCL_TYPEDEFS_H_
#define PCL_TYPEDEFS_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointNormal PointNormal;

typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

typedef pcl::PointCloud<PointNormal> PointCloudNormals;
typedef PointCloudNormals::Ptr PointCloudNormalsPtr;
typedef PointCloudNormals::ConstPtr PointCloudNormalsConstPtr;

#endif
