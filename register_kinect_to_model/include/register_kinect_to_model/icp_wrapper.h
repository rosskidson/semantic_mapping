/*
 * icp_wrapper.h
 *
 *  Created on: Nov 8, 2012
 *      Author: kidson
 */

#ifndef ICP_WRAPPER_H_
#define ICP_WRAPPER_H_

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

// pcl typedefs
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormals;
typedef PointCloudNormals::Ptr PointCloudNormalsPtr;
typedef PointCloudNormals::ConstPtr PointCloudNormalsConstPtr;

class ICPWrapper
{
  public:
    ICPWrapper ();
    virtual ~ICPWrapper ();
    Eigen::Matrix4f performICP (PointCloudConstPtr source_cloud_ptr,
        PointCloudConstPtr target_cloud_ptr, Eigen::Matrix4f initial_transform);
  private:
    pcl::IterativeClosestPoint<PointType, PointType> icp_;
};

#endif /* ICP_WRAPPER_H_ */
