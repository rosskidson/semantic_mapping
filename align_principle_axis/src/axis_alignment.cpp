/*
 * axis_alignment.cpp
 *
 *  Created on: Nov 18, 2012
 *      Author: kidson
 */

#include "align_principle_axis/axis_alignment.h"

#include "pcl_typedefs/pcl_typedefs.h"

#include <pcl/common/transforms.h>

#include <ros/console.h>

namespace align_principle_axis
{

  AxisAlignment::AxisAlignment ()
  {
    // TODO Auto-generated constructor stub

  }

  AxisAlignment::~AxisAlignment ()
  {
    // TODO Auto-generated destructor stub
  }

  void AxisAlignment::moveModelToOrigin (const PointCloudConstPtr cloud_input_ptr,
      const PointCloudPtr cloud_output_ptr,
      Eigen::Matrix4f& transform_output)
  {
    //find min in x,y and z.  Move this to Origin
    float min_x, min_y, min_z;
    min_x = min_y = min_z = std::numeric_limits<float>::max ();
    for (std::vector<PointType, Eigen::aligned_allocator<PointType> >::const_iterator itr =
        cloud_input_ptr->points.begin ();
        itr != cloud_input_ptr->points.end (); itr++)
    {
      if (itr->x < min_x)
        min_x = itr->x;
      if (itr->y < min_y)
        min_y = itr->y;
      if (itr->z < min_z)
        min_z = itr->z;
    }
    transform_output = Eigen::Matrix4f::Identity (4, 4);
    transform_output (0, 3) = -min_x;
    transform_output (1, 3) = -min_y;
    transform_output (2, 3) = -min_z;
    transformPointCloud (*cloud_input_ptr, *cloud_output_ptr, transform_output);
  }
}
