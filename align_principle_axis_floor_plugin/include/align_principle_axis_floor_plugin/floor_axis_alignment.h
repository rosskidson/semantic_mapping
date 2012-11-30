/*
 * floor_axis_alignment.h
 *
 *  Created on: 20/11/2012
 *      Author: ross
 */

#ifndef FLOOR_AXIS_ALIGNMENT_H_
#define FLOOR_AXIS_ALIGNMENT_H_

#include "align_principle_axis_interface/axis_alignment.h"

namespace align_principle_axis_floor_plugin
{

  class FloorAxisAlignment : public align_principle_axis_interface::AxisAlignment
  {
    public:
      FloorAxisAlignment ();
      virtual ~FloorAxisAlignment ();

      virtual void alignCloudPrincipleAxis (const PointCloudConstPtr input_cloud_ptr,
          const Eigen::Matrix4f& inital_guess, const PointCloudPtr output_cloud_ptr,
          Eigen::Matrix4f& transform_output);
  };

}
#endif /* FLOOR_AXIS_ALIGNMENT_H_ */
