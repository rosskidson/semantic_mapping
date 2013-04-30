/*
 * FixtureSegmentationFromPlanes.h
 *
 *  Created on: Dec 11, 2012
 *      Author: kidson
 */

#ifndef FIXTURESEGMENTATIONFROMPLANES_H_
#define FIXTURESEGMENTATIONFROMPLANES_H_

#include "ros/ros.h"
#include <pcl_typedefs/pcl_typedefs.h>
#include <segment_fixtures_interface/fixture_segmentation.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>


#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/segment_fixtures_from_planes_plugin/FixtureSegmentationConfig.h"

namespace segment_fixtures_from_planes_plugin
{
  class FixtureSegmentationFromPlanes : public segment_fixtures_interface::FixtureSegmentation
  {
    public:
      FixtureSegmentationFromPlanes();
      virtual ~FixtureSegmentationFromPlanes();

      virtual void segmentFixtures(const PointCloudConstPtr model, std::vector<pcl::PointIndicesConstPtr>& fixture_indices_ptrs);

      virtual void setPlanes(std::vector<pcl::PointIndicesConstPtr>& plane_indices_ptrs,
            std::vector<pcl::ModelCoefficients::ConstPtr>& plane_coeffs_);

    private:

      void reconfigCallback (segment_fixtures_from_planes_plugin::FixtureSegmentationConfig &config, uint32_t level);

      std::vector<pcl::PointIndicesConstPtr> plane_indices_ptrs_;
      std::vector<pcl::ModelCoefficients::ConstPtr> plane_coeffs_;

      ros::NodeHandle nh_;
      dynamic_reconfigure::Server<segment_fixtures_from_planes_plugin::FixtureSegmentationConfig> reconfig_srv_;
      dynamic_reconfigure::Server<segment_fixtures_from_planes_plugin::FixtureSegmentationConfig>::CallbackType reconfig_callback_;

      int min_handle_candidates_points_;
      float plane_scale_down_factor_;

      pcl::ProjectInliers<PointType> projector_; // project points from planes onto a perfect plane
      pcl::ConvexHull<PointType> chull_;
      pcl::ExtractPolygonalPrismData<PointType> prism_;
      pcl::EuclideanClusterExtraction<PointType> fixture_cluster_;
      pcl::search::KdTree<PointType>::Ptr clusters_tree_ptr_;

  };
}

#endif /* FIXTURESEGMENTATIONFROMPLANES_H_ */
