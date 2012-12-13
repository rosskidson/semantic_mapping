/*
 * FixtureSegmentationFromPlanes.cpp
 *
 *  Created on: Dec 11, 2012
 *      Author: ross kidson
 */

#include "segment_fixtures_from_planes_plugin/fixture_segmentation_from_planes.h"

#include <pcl17/filters/project_inliers.h>
#include <pcl17/segmentation/extract_polygonal_prism_data.h>
#include <pcl17/segmentation/extract_clusters.h>
#include <pcl17/surface/convex_hull.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/filters/extract_indices.h>

#include <ros/console.h>

//pluginlib
#include <pluginlib/class_list_macros.h>

//Declare the plugin
//param_1: The namespace in which the  plugin will live
//param_2: The name we wish to give to the plugin
// loader.createClassInstance("param_1/param_2");
//param_3: The fully-qualified type of the plugin class
//param_4: The fully-qualified type of the base class
PLUGINLIB_DECLARE_CLASS(segment_fixtures_from_planes_plugin, FixtureSegmentationFromPlanes, segment_fixtures_from_planes_plugin::FixtureSegmentationFromPlanes, segment_fixtures_interface::FixtureSegmentation)

namespace segment_fixtures_from_planes_plugin
{

  FixtureSegmentationFromPlanes::FixtureSegmentationFromPlanes ():
    nh_("~/fixture_segmentation_from_planes"),
    reconfig_srv_(nh_),
    projector_(),
    chull_(),
    prism_(),
    fixture_cluster_(),
    clusters_tree_ptr_(new pcl17::search::KdTree<PointType>)
  {
    clusters_tree_ptr_->setEpsilon(1);

    reconfig_callback_ = boost::bind (&FixtureSegmentationFromPlanes::reconfigCallback, this, _1, _2);
    reconfig_srv_.setCallback (reconfig_callback_);
  }

  FixtureSegmentationFromPlanes::~FixtureSegmentationFromPlanes ()
  {
    // TODO Auto-generated destructor stub
  }

  void FixtureSegmentationFromPlanes::reconfigCallback (segment_fixtures_from_planes_plugin::FixtureSegmentationConfig &config, uint32_t level)
  {
    //  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
    //            config.int_param, config.double_param,
    //            config.str_param.c_str(),
    //            config.bool_param?"True":"False",
    //            config.size);

    chull_.setComputeAreaVolume(config.convex_hull_compute_area_volume);
    prism_.setHeightLimits(config.min_search_dist_from_plane, config.max_search_dist_from_plane);

    fixture_cluster_.setClusterTolerance(config.fixture_cluster_tolerance);
    fixture_cluster_.setMaxClusterSize(config.max_fixture_cluster_size);
    fixture_cluster_.setMinClusterSize(config.min_fixture_cluster_size);

    min_handle_candidates_points_ = config.min_fixture_points;

  }

  void FixtureSegmentationFromPlanes::setPlanes(std::vector<PointCloudConstPtr>& plane_clouds,
                                                std::vector<pcl17::ModelCoefficients::ConstPtr>& plane_coeffs)
  {
    plane_clouds_ = plane_clouds;
    plane_coeffs_ = plane_coeffs;
  }

  void FixtureSegmentationFromPlanes::segmentFixtures(const PointCloudConstPtr model, std::vector<PointCloudConstPtr>& fixture_cloud_ptrs)
  {
    ROS_INFO("segment fixtures from planes");
    for (uint plane_num = 0; plane_num < plane_clouds_.size(); plane_num++)
    {
      ROS_INFO("Plane model: [%f, %f, %f, %f] with %d inliers.",
               plane_coeffs_[plane_num]->values[0], plane_coeffs_[plane_num]->values[1],
               plane_coeffs_[plane_num]->values[2], plane_coeffs_[plane_num]->values[3], plane_clouds_.size());

      //Project Points into a perfect plane
      PointCloudPtr cloud_projected(new PointCloud());
      projector_.setInputCloud(model);
      projector_.setModelCoefficients(plane_coeffs_[plane_num]);
      projector_.setModelType(pcl17::SACMODEL_PARALLEL_PLANE);
      projector_.filter(*cloud_projected);

      // Create a Convex Hull representation of the projected inliers
      PointCloud::Ptr cloud_hull(new PointCloud());
      chull_.setInputCloud(cloud_projected);
      chull_.reconstruct(*cloud_hull);
      ROS_INFO("Convex hull has: %d data points.", (int)cloud_hull->points.size ());
      if ((int) cloud_hull->points.size() == 0)
      {
        ROS_WARN("Convex hull has: no points. Returning.");
        return;
      }

      // Extract the handle clusters using a polygonal prism
      pcl17::PointIndices::Ptr handles_indices_ptr(new pcl17::PointIndices());

      prism_.setInputCloud(model);
      prism_.setInputPlanarHull(cloud_hull);
      prism_.segment(*handles_indices_ptr);
      ROS_INFO("Number of handle candidates: %d.", (int)handles_indices_ptr->indices.size ());
      if((int)handles_indices_ptr->indices.size () < min_handle_candidates_points_)
        continue;

      //######### handle clustering code
      std::vector<pcl17::PointIndices> handle_clusters;
      fixture_cluster_.setSearchMethod(clusters_tree_ptr_);
      fixture_cluster_.setInputCloud(model);
      fixture_cluster_.setIndices(handles_indices_ptr);
      fixture_cluster_.extract(handle_clusters);

      for(std::vector<pcl17::PointIndices>::const_iterator itr=handle_clusters.begin(); itr!= handle_clusters.end(); itr++)
      {
        pcl17::ExtractIndices<PointType> filter;
        PointCloudPtr fixture_ptr (new PointCloud);
        filter.setInputCloud(model);
        pcl17::PointIndicesPtr indices_ptr (new pcl17::PointIndices(*itr));
        filter.setIndices(indices_ptr);
        filter.filter(*fixture_ptr);
        fixture_cloud_ptrs.push_back(fixture_ptr);
      }
    }
  }
}
