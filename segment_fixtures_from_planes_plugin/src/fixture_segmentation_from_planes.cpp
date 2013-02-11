/*
 * FixtureSegmentationFromPlanes.cpp
 *
 *  Created on: Dec 11, 2012
 *      Author: ross kidson
 */

#include <stdio.h>

#include "segment_fixtures_from_planes_plugin/fixture_segmentation_from_planes.h"

#include <pcl17/filters/project_inliers.h>
#include <pcl17/segmentation/extract_polygonal_prism_data.h>
#include <pcl17/segmentation/extract_clusters.h>
#include <pcl17/surface/convex_hull.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/filters/extract_indices.h>
#include <pcl17/common/centroid.h>

#include <ros/console.h>

//#include <visualizer/visualization.h>
#include <pcl_tools/pcl_tools.h>

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
    plane_scale_down_factor_ = config.plane_scale_down_factor;

  }

  void FixtureSegmentationFromPlanes::setPlanes(std::vector<pcl17::PointIndicesConstPtr>& plane_indices_ptrs,
                                                std::vector<pcl17::ModelCoefficients::ConstPtr>& plane_coeffs)
  {
    plane_indices_ptrs_ = plane_indices_ptrs;
    plane_coeffs_ = plane_coeffs;
  }

  void FixtureSegmentationFromPlanes::segmentFixtures(const PointCloudConstPtr model, std::vector<pcl17::PointIndicesConstPtr>& fixture_indices_ptrs)
  {
    ROS_INFO("segment fixtures from planes");

    // first create a new model with the planes substracted
    PointCloudPtr model_noplanes_ptr (new PointCloud);
    pcl17::PointIndicesPtr indices_ptr (new pcl17::PointIndices);
    for(std::vector<pcl17::PointIndicesConstPtr>::const_iterator plane_itr = plane_indices_ptrs_.begin(); plane_itr != plane_indices_ptrs_.end(); plane_itr++)
      for(std::vector<int>::const_iterator itr = (**plane_itr).indices.begin(); itr != (**plane_itr).indices.end(); itr++)
        indices_ptr->indices.push_back(*itr);
    // remove duplicates
    std::sort(indices_ptr->indices.begin(), indices_ptr->indices.end());
    indices_ptr->indices.erase(std::unique(indices_ptr->indices.begin(), indices_ptr->indices.end()),indices_ptr->indices.end());

    pcl17::ExtractIndices<PointType> extractor;
    extractor.setNegative(true);
    extractor.setIndices(indices_ptr);
    extractor.setInputCloud(model);
    extractor.filter(*model_noplanes_ptr);

    //Visualization vis;
    //vis.addCloudToVisualizer(model_noplanes_ptr);

    extractor.setNegative(false);
    for (uint plane_num = 0; plane_num < plane_coeffs_.size(); plane_num++)
    {
      //little hack for now to get the planes I want.
      if(plane_num !=6 && plane_num !=8 )
        continue;
      ROS_DEBUG("Plane model: [%f, %f, %f, %f] with %d inliers.",
               plane_coeffs_[plane_num]->values[0], plane_coeffs_[plane_num]->values[1],
               plane_coeffs_[plane_num]->values[2], plane_coeffs_[plane_num]->values[3], plane_indices_ptrs_[plane_num]->indices.size());

      //Project Points into a perfect plane
      PointCloudPtr cloud_projected(new PointCloud());
      projector_.setInputCloud(model);
      projector_.setIndices(plane_indices_ptrs_[plane_num]);
      projector_.setModelCoefficients(plane_coeffs_[plane_num]);
      projector_.setModelType(pcl17::SACMODEL_PARALLEL_PLANE);
      projector_.filter(*cloud_projected);

      // Create a Convex Hull representation of the projected inliers
      PointCloud::Ptr cloud_hull(new PointCloud());
      chull_.setInputCloud(cloud_projected);
      chull_.reconstruct(*cloud_hull);

      //move hull pointcloud to center
      PointCloud::Ptr centered_cloud_hull(new PointCloud());
      Eigen::Vector4d hull_centroid_vec;
      pcl17::compute3DCentroid(*cloud_hull, hull_centroid_vec);
      Eigen::Matrix4f hull_centroid_mat = Eigen::Matrix4f::Identity();
      for(int i=0; i<4; i++)
        hull_centroid_mat(i,3) = -hull_centroid_vec(i);
      pcl_tools::transformPointCloud(cloud_hull, centered_cloud_hull, hull_centroid_mat);
      // scale it down
      PointCloud::Ptr resized_cloud_hull(new PointCloud());
      pcl_tools::transformPointCloud(centered_cloud_hull, resized_cloud_hull, plane_scale_down_factor_* Eigen::Matrix4f::Identity());
      //move it back
      for(int i=0; i<4; i++)
        hull_centroid_mat(i,3) = hull_centroid_vec(i);
      pcl_tools::transformPointCloud(resized_cloud_hull, cloud_hull, hull_centroid_mat);

      PointCloudPtr temp (new PointCloud());
      extractor.setIndices(plane_indices_ptrs_[plane_num]);
      extractor.filter(*temp);

//      vis.addCloudToVisualizer(temp);
//      ros::Duration(0.5).sleep();
//      vis.addCloudToVisualizer(cloud_hull);
//      ros::Duration(0.5).sleep();
//      std::vector<PointCloudConstPtr> vis_clouds;
//      vis.addCloudToVisualizer(vis_clouds);

      ROS_DEBUG("Convex hull has: %d data points.", (int)cloud_hull->points.size ());
      if ((int) cloud_hull->points.size() == 0)
      {
        ROS_WARN("Convex hull has: no points. Returning.");
        return;
      }

      // Extract the handle clusters using a polygonal prism
      pcl17::PointIndices::Ptr handles_indices_ptr(new pcl17::PointIndices());

      prism_.setInputCloud(model_noplanes_ptr);
      prism_.setInputPlanarHull(cloud_hull);
      prism_.segment(*handles_indices_ptr);
      ROS_INFO("Number of handle candidates: %d.", (int)handles_indices_ptr->indices.size ());
      if((int)handles_indices_ptr->indices.size () < min_handle_candidates_points_)
        continue;

      //######### handle clustering code
      std::vector<pcl17::PointIndices> handle_clusters;
      fixture_cluster_.setSearchMethod(clusters_tree_ptr_);
      fixture_cluster_.setInputCloud(model_noplanes_ptr);
      fixture_cluster_.setIndices(handles_indices_ptr);
      fixture_cluster_.extract(handle_clusters);

      std::vector<pcl17::PointIndicesConstPtr> debug;
      for(std::vector<pcl17::PointIndices>::const_iterator itr=handle_clusters.begin(); itr!= handle_clusters.end(); itr++)
      {
        pcl17::PointIndicesPtr indices_ptr (new pcl17::PointIndices(*itr));
        pcl17::PointIndicesPtr model_indices_ptr = pcl_tools::getIndicesFromPointCloud(model_noplanes_ptr, indices_ptr, model);
        debug.push_back(model_indices_ptr);
        fixture_indices_ptrs.push_back(model_indices_ptr);
      }

//      vis.addCloudsToVisualizer(model, debug);
//      ros::Duration(0.5).sleep();
    }
  }
}
