/*
 * box_filter.h
 *
 *  Created on: Nov 17, 2012
 *      Author: kidson
 */

#ifndef BOX_FILTER_H_
#define BOX_FILTER_H_

#include "pcl_typedefs/pcl_typedefs.h"

#include <Eigen/Core>

namespace box_filter
{

void filterCloud (const PointCloudConstPtr input_cloud_ptr, const Eigen::Vector4f& min_point,
    const Eigen::Vector4f& max_point, const PointCloudPtr output_cloud_ptr);
}

#endif /* BOX_FILTER_H_ */
