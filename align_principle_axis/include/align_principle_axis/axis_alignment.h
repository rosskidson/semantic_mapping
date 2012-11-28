/*
 * axis_alignment.h
 *
 *  Created on: Nov 18, 2012
 *      Author: kidson
 */

#ifndef AXIS_ALIGNMENT_H_
#define AXIS_ALIGNMENT_H_

#include "pcl_typedefs/pcl_typedefs.h"

#include <Eigen/Core>

namespace align_principle_axis
{
  class AxisAlignment
  {
    public:
      AxisAlignment ();
      virtual ~AxisAlignment ();

      virtual void alignCloudPrincipleAxis (const PointCloudConstPtr cloud_input,
          const Eigen::Matrix4f& inital_guess,
          const PointCloudPtr cloud_output, Eigen::Matrix4f& transform_output) = 0;

      /*virtual*/ void moveModelToOrigin (const PointCloudConstPtr cloud_input_ptr,
          const PointCloudPtr cloud_output_ptr,
          Eigen::Matrix4f& transform_output);
  };
}

#include "../../src/axis_alignment.cpp"

#endif /* AXIS_ALIGNMENT_H_ */
