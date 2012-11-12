/*
 * icp_wrapper.h
 *
 *  Created on: Nov 8, 2012
 *      Author: kidson
 */

#ifndef ICP_WRAPPER_H_
#define ICP_WRAPPER_H_


#include <pcl/registration/icp.h>
#include "register_kinect_to_model/pcl_typedefs.h"

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
