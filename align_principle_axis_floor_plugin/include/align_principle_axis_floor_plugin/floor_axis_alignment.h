/*
 * floor_axis_alignment.h
 *
 *  Created on: 20/11/2012
 *      Author: ross
 */

#ifndef FLOOR_AXIS_ALIGNMENT_H_
#define FLOOR_AXIS_ALIGNMENT_H_

#include "align_principle_axis_interface/axis_alignment.h"

#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/align_principle_axis_floor_plugin/FloorAxisAlignmentConfig.h"

namespace align_principle_axis_floor_plugin
{

  class FloorAxisAlignment : public align_principle_axis_interface::AxisAlignment
  {
    public:
      FloorAxisAlignment ();
      virtual ~FloorAxisAlignment ();

      virtual void alignCloudPrincipleAxis (const PointCloudConstPtr input_cloud_ptr,
          const PointCloudPtr output_cloud_ptr,
          Eigen::Matrix4f& transform_output);

  private:

      void reconfigCallback (align_principle_axis_floor_plugin::FloorAxisAlignmentConfig &config, uint32_t level);

      ros::NodeHandle nh_;
      dynamic_reconfigure::Server<align_principle_axis_floor_plugin::FloorAxisAlignmentConfig> reconfig_srv_;

      // local params
      Eigen::Vector3f axis_;
      double angle_;
      double ransac_threshold_;
      int max_iterations_;

  };

}
#endif /* FLOOR_AXIS_ALIGNMENT_H_ */
