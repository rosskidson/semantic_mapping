/*
 * PlaneSegmentationRANSAC.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: kidson
 */

#include "segment_planes_ransac_plugin/plane_segmentation_ransac.h"

#include "pcl/segmentation/organized_multi_plane_segmentation.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>

#include <ros/console.h>

//pluginlib
#include <pluginlib/class_list_macros.h>

//Declare the plugin
//param_1: The namespace in which the  plugin will live
//param_2: The name we wish to give to the plugin
// loader.createClassInstance("param_1/param_2");
//param_3: The fully-qualified type of the plugin class
//param_4: The fully-qualified type of the base class
PLUGINLIB_DECLARE_CLASS(segment_planes_ransac_plugin, PlaneSegmentationRANSAC, segment_planes_ransac_plugin::PlaneSegmentationRANSAC, segment_planes_interface::PlaneSegmentation)

namespace segment_planes_ransac_plugin
{

  PlaneSegmentationRANSAC::PlaneSegmentationRANSAC ()
  {
    // TODO Auto-generated constructor stub

  }

  PlaneSegmentationRANSAC::~PlaneSegmentationRANSAC ()
  {
    // TODO Auto-generated destructor stub
  }

  void calculatePointCloudNormals (const PointCloudConstPtr input_cloud_ptr,
      PointCloudNormalsPtr cloud_normals)
  {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointType, PointNormal> ne;
    //ne.setNumberOfThreads(4);
    ne.setInputCloud (input_cloud_ptr);

    pcl::search::KdTree<PointType>::Ptr normals_tree (new pcl::search::KdTree<PointType>);
    //ne.setKSearch(30);
    ne.setRadiusSearch(0.1);
    ne.setSearchMethod(normals_tree);
    ne.compute (*cloud_normals);
  }

  void PlaneSegmentationRANSAC::segmentPlanes (const PointCloudConstPtr model,
      const std::vector<PointCloudConstPtr>& plane_clouds, const std::vector<
          pcl::ModelCoefficients::ConstPtr>& plane_coeffs)
  {
    //calculate normals
    PointCloudNormalsPtr cloud_normals (new PointCloudNormals);
    calculatePointCloudNormals(model,cloud_normals);

    pcl::OrganizedMultiPlaneSegmentation<PointType, PointNormal, pcl::Label> mps;
    //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals;

    ROS_INFO("Segmenting planes");
    mps.setMinInliers (20000);
    mps.setMaximumCurvature(0.02);
    mps.setInputNormals (cloud_normals);
    mps.setInputCloud (model);
    std::vector<pcl::PlanarRegion<PointType> > regions;
    std::vector<pcl::PointIndices> regionPoints;
    std::vector< pcl::ModelCoefficients > planes_coeff;
    mps.segment(planes_coeff, regionPoints);
    ROS_INFO_STREAM("Number of regions:" << regionPoints.size());

    if ((int) regionPoints.size() < 1) {
      ROS_ERROR("no planes found");
      return;
    }

      std::stringstream filename;
    for (size_t plane = 0; plane < regionPoints.size (); plane++)
    {
      //filename.str("");
      //filename << "plane" << plane << ".pcd";
      //writer.write(filename.str(), *cloudInput, regionPoints[plane].indices, true);
      ROS_INFO("Plane model: [%f, %f, %f, %f] with %d inliers.",
          planes_coeff[plane].values[0], planes_coeff[plane].values[1],
          planes_coeff[plane].values[2], planes_coeff[plane].values[3], (int)regionPoints[plane].indices.size ());
    }
  }

}
