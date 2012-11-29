/*
 * pcl_tools.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: ross kidson
 */

#include "pcl_tools/pcl_tools.h"

#include <pcl_typedefs/pcl_typedefs.h>

//normal estimation
#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/features/normal_3d_omp.h>
//voxel grid filter
#include <pcl17/filters/voxel_grid.h>
// tranform cloud
#include <pcl/common/transforms.h>



namespace pcl_tools
{




  void calculateNormals (PointCloudConstPtr input_cloud_ptr, PointCloudNormalsPtr normals_ptr)
  {
    pcl17::NormalEstimation<PointType, PointNormal> normal_est;
    normal_est.setInputCloud (input_cloud_ptr);
    pcl17::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    normal_est.setSearchMethod (tree);
    normal_est.setRadiusSearch (0.05);
    normal_est.compute (*normals_ptr);
    //pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
  }

  void calculateNormalsOMP (PointCloudConstPtr input_cloud_ptr, PointCloudNormalsPtr normals_ptr)
  {
    pcl17::NormalEstimationOMP<PointType, PointNormal> normal_est;
    normal_est.setInputCloud (input_cloud_ptr);
    pcl17::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    normal_est.setSearchMethod (tree);
    normal_est.setRadiusSearch (0.05);
    normal_est.compute (*normals_ptr);
    //pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
  }

  void transformPointCloud(PointCloudConstPtr input_cloud_ptr, PointCloudPtr output_cloud_ptr, const Eigen::Affine3f &transform )
  {
   transformPointCloud (*input_cloud_ptr, *output_cloud_ptr, transform);
  }

}
