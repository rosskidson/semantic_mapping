/*
 * icp_wrapper.h
 *
 *  Created on: Nov 8, 2012
 *      Author: kidson
 */

#ifndef ICP_WRAPPER_H_
#define ICP_WRAPPER_H_


#include <pcl17/registration/icp.h>
#include "pcl_typedefs/pcl_typedefs.h"

class ICPWrapper
{
  public:
    ICPWrapper ();
    virtual ~ICPWrapper ();
    Eigen::Matrix4f performICP (PointCloudConstPtr source_cloud_ptr,
        PointCloudConstPtr target_cloud_ptr, Eigen::Matrix4f initial_transform);
  private:
    PointCloudConstPtr downsampleCloud (PointCloudConstPtr input);

    pcl17::IterativeClosestPoint<PointType, PointType> icp_;
};

#endif /* ICP_WRAPPER_H_ */
