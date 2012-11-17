/*
 * box_filter.cpp
 *
 *  Created on: Nov 11, 2012
 *      Author: kidson
 */

#include "box_filter/box_filter.h"
#include <pcl/filters/crop_box.h>

namespace box_filter
{

  void filterCloud (const PointCloudConstPtr input_cloud_ptr, const Eigen::Vector4f& min_point,
      const Eigen::Vector4f& max_point, const PointCloudPtr output_cloud_ptr)
  {
    // apply filter
    pcl::CropBox<PointType> box_filter;
    box_filter.setInputCloud (input_cloud_ptr);
    box_filter.setMin (min_point);
    box_filter.setMax (max_point);
    box_filter.filter (*output_cloud_ptr);

  }

}
