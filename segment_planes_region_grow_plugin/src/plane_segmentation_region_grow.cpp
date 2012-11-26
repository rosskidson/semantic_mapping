/*
 * PlaneSegmentationRANSAC.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: kidson
 */

#include "segment_planes_region_grow_plugin/plane_segmentation_region_grow.h"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>



#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

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

  PlaneSegmentationRegionGrow::PlaneSegmentationRegionGrow ()
  {
    // TODO Auto-generated constructor stub

  }

  PlaneSegmentationRegionGrow::~PlaneSegmentationRegionGrow ()
  {
    // TODO Auto-generated destructor stub
  }

  void calculatePointCloudNormals (const PointCloudConstPtr input_cloud_ptr,
      PointCloudNormalsPtr cloud_normals)
  {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<PointType, PointNormal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud (input_cloud_ptr);

    pcl::search::KdTree<PointType>::Ptr normals_tree (new pcl::search::KdTree<PointType>);
    //ne.setKSearch(30);
    ne.setRadiusSearch(0.1);
    ne.setSearchMethod(normals_tree);
    ne.compute (*cloud_normals);
  }

  void PlaneSegmentationRegionGrow::segmentPlanes (const PointCloudConstPtr model,
      const std::vector<PointCloudConstPtr>& plane_clouds, const std::vector<
          pcl::ModelCoefficients::ConstPtr>& plane_coeffs)
  {
    ROS_INFO("region_grow");
    //calculate normals
    PointCloudNormalsPtr cloud_normals (new PointCloudNormals);
    calculatePointCloudNormals(model,cloud_normals);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (10000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (7.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
      std::endl << "cloud that belong to the first cluster:" << std::endl;
    int counter = 0;
    while (counter < 5 || counter > clusters[0].indices.size ())
    {
      std::cout << clusters[0].indices[counter] << std::endl;
      counter++;
    }

  }

}
