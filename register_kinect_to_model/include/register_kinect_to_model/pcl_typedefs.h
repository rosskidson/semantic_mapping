#ifndef PCL_TYPEDEFS_H_
#define PCL_TYPEDEFS_H_


#include <pcl/point_types.h>

// pcl typedefs
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormals;
typedef PointCloudNormals::Ptr PointCloudNormalsPtr;
typedef PointCloudNormals::ConstPtr PointCloudNormalsConstPtr;


#endif /* PCL_TYPEDEFS_H_ */
