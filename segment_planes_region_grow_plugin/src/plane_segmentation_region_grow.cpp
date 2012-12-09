/*
 * PlaneSegmentationRANSAC.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: kidson
 */

#include "segment_planes_region_grow_plugin/plane_segmentation_region_grow.h"


#include <pcl17/search/search.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/segmentation/region_growing.h>

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
PLUGINLIB_DECLARE_CLASS(segment_planes_region_grow_plugin, PlaneSegmentationRegionGrow, segment_planes_region_grow_plugin::PlaneSegmentationRegionGrow, segment_planes_interface::PlaneSegmentation)

namespace segment_planes_region_grow_plugin
{

  PlaneSegmentationRegionGrow::PlaneSegmentationRegionGrow ():
    normals_ptr_(new pcl17::PointCloud<pcl17::Normal>),
    nh_("~/plane_segmentation_region_grow"),
    reconfig_srv_(nh_),
    region_grow_()
  {
    reconfig_callback_ = boost::bind (&PlaneSegmentationRegionGrow::reconfigCallback, this, _1, _2);
    reconfig_srv_.setCallback (reconfig_callback_);
  }

  PlaneSegmentationRegionGrow::~PlaneSegmentationRegionGrow ()
  {
    // TODO Auto-generated destructor stub
  }

  void PlaneSegmentationRegionGrow::reconfigCallback (segment_planes_region_grow_plugin::PlaneSegmentationConfig &config,
      uint32_t level)
  {
    //  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
    //            config.int_param, config.double_param,
    //            config.str_param.c_str(),
    //            config.bool_param?"True":"False",
    //            config.size);

    region_grow_.setCurvatureTestFlag(config.curvature_test_flag);
    region_grow_.setCurvatureThreshold(config.curvature_threshold);
    region_grow_.setMaxClusterSize(config.max_cluster_size);
    region_grow_.setMinClusterSize(config.min_cluster_size);
    region_grow_.setNumberOfNeighbours(config.number_of_neighbours);
    region_grow_.setResidualTestFlag(config.residual_test_flag);
    region_grow_.setResidualThreshold(config.residual_threshold);
    region_grow_.setSmoothModeFlag(config.smooth_mode_flag);
    region_grow_.setSmoothnessThreshold(config.smoothness_threshold);
  }

  void PlaneSegmentationRegionGrow::setNormals(const PointCloudNormalsConstPtr normals)
  {
    pcl17::copyPointCloud(*normals, *normals_ptr_);
  }

  void PlaneSegmentationRegionGrow::segmentPlanes (const PointCloudConstPtr model,
      std::vector<PointCloudConstPtr>& plane_clouds, std::vector<
          pcl17::ModelCoefficients::ConstPtr>& plane_coeffs)
  {
    ROS_INFO("region_grow");
    //calculate normals
    if(!normals_ptr_)
    {
      ROS_WARN("normals for segmentPlanes object not set. Aborting");
      return;
    }

    // region growing only works for pointXYZ.  convert here
    pcl17::PointCloud<pcl17::PointXYZ>::Ptr model_noRGB_ptr (new pcl17::PointCloud<pcl17::PointXYZ>);
    pcl17::copyPointCloud(*model, *model_noRGB_ptr);

    pcl17::search::Search<pcl17::PointXYZ>::Ptr tree = boost::shared_ptr<pcl17::search::Search<pcl17::PointXYZ> > (new pcl17::search::KdTree<pcl17::PointXYZ>);

    region_grow_.setMinClusterSize (100);
    region_grow_.setMaxClusterSize (10000);
    region_grow_.setSearchMethod (tree);
    region_grow_.setNumberOfNeighbours (30);
    region_grow_.setInputCloud (model_noRGB_ptr);
    region_grow_.setInputNormals (normals_ptr_);
    region_grow_.setSmoothnessThreshold (7.0 / 180.0 * M_PI);
    region_grow_.setCurvatureThreshold (1.0);

    std::vector <pcl17::PointIndices> clusters;
    region_grow_.extract ( clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points.\n";
    std::cout << "These are the indices of the points of the initial" <<
      std::endl << "cloud that belong to the first cluster:" << std::endl;
    for(std::vector<pcl17::PointIndices>::iterator itr = clusters.begin(); itr != clusters.end(); itr++)
    {
      pcl17::ExtractIndices<PointType> filter;
      PointCloudPtr plane_ptr (new PointCloud);
      filter.setInputCloud(model);
      pcl17::PointIndicesPtr indices_ptr (new pcl17::PointIndices);
      for(std::vector<int>::iterator idx_itr = itr->indices.begin(); idx_itr != itr->indices.end(); idx_itr++)
        indices_ptr->indices.push_back(*idx_itr);
      filter.setIndices(indices_ptr);
      filter.filter(*plane_ptr);
      plane_clouds.push_back(plane_ptr);
    }

  }

}
