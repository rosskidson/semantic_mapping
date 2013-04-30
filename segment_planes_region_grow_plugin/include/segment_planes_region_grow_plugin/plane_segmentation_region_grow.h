/*
 * PlaneSegmentationRANSAC.h
 *
 *  Created on: Nov 26, 2012
 *      Author: kidson
 */

#ifndef PLANESEGMENTATIONREGIONGROW_H_
#define PLANESEGMENTATIONREGIONGROW_H_

#include "ros/ros.h"
#include <segment_planes_interface/plane_segmentation.h>

#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/segment_planes_region_grow_plugin/PlaneSegmentationConfig.h"

#include <pcl/segmentation/region_growing.h>

namespace segment_planes_region_grow_plugin
{
  class PlaneSegmentationRegionGrow : public segment_planes_interface::PlaneSegmentation
  {
    public:
      PlaneSegmentationRegionGrow();
      virtual ~PlaneSegmentationRegionGrow();

      virtual void segmentPlanes(const PointCloudConstPtr model,
          std::vector<pcl::PointIndicesConstPtr>& plane_indices_ptrs,
          std::vector<pcl::ModelCoefficients::ConstPtr>& plane_coeffs);

      virtual void setNormals(const PointCloudNormalsConstPtr normals);

    private:

      void reconfigCallback (segment_planes_region_grow_plugin::PlaneSegmentationConfig &config,
          uint32_t level);

      pcl::PointCloud<pcl::Normal>::Ptr normals_ptr_;

      ros::NodeHandle nh_;
      dynamic_reconfigure::Server<segment_planes_region_grow_plugin::PlaneSegmentationConfig> reconfig_srv_;
      dynamic_reconfigure::Server<segment_planes_region_grow_plugin::PlaneSegmentationConfig>::CallbackType
          reconfig_callback_;

      pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> region_grow_;

      bool curvature_test_flag_, residual_test_flag_, smooth_mode_flag_;
      float curvature_threshold_, residual_threshold_, smoothness_threshold_;
      int max_cluster_size_, min_cluster_size_, number_of_neighbours_;


  };
}

#endif /* PLANESEGMENTATIONREGIONGROW_H_ */
