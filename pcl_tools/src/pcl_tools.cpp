/*
 * pcl_tools.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: ross kidson
 */

#include "pcl_tools/pcl_tools.h"

#include <pcl_typedefs/pcl_typedefs.h>

#include <Eigen/Core>

//normal estimation
#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/features/normal_3d_omp.h>
//voxel grid filter
#include <pcl17/filters/voxel_grid.h>
//box filter
#include <pcl17/filters/crop_box.h>
// tranform cloud
#include <pcl17/common/transforms.h>
// extract indices
#include <pcl17/filters/extract_indices.h>
// kdtree
#include <pcl17/kdtree/kdtree_flann.h>

// conversions
#include "cv_bridge/cv_bridge.h"
#include "pcl17/ros/conversions.h"

namespace pcl_tools
{
  void calculateNormals (PointCloudConstPtr input_cloud_ptr, PointCloudNormalsPtr normals_ptr)
  {
    pcl17::NormalEstimation<PointType, PointNormal> normal_est;
    normal_est.setInputCloud (input_cloud_ptr);
    pcl17::search::KdTree<PointType>::Ptr tree (new pcl17::search::KdTree<PointType> ());
    normal_est.setSearchMethod (tree);
    normal_est.setKSearch(50);
    normal_est.compute (*normals_ptr);
    //pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
  }

  void calculateNormalsOMP (PointCloudConstPtr input_cloud_ptr, PointCloudNormalsPtr normals_ptr)
  {
    pcl17::NormalEstimationOMP<PointType, PointNormal> normal_est;
    normal_est.setInputCloud (input_cloud_ptr);
    pcl17::search::KdTree<PointType>::Ptr tree (new pcl17::search::KdTree<PointType> ());
    normal_est.setSearchMethod (tree);
    normal_est.setKSearch(50);
    normal_est.compute (*normals_ptr);
    //pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
  }

  void transformPointCloud(PointCloudConstPtr input_cloud_ptr, PointCloudPtr output_cloud_ptr, const Eigen::Matrix4f& transform )
  {
   transformPointCloud (*input_cloud_ptr, *output_cloud_ptr, transform);
  }

  // searches for points from a given point cloud in another pointcloud to convert a pointcloud to indices
  pcl17::PointIndicesPtr getIndicesFromPointCloud(const PointCloudConstPtr& input_cloud_ptr,
                                                  const pcl17::PointIndicesConstPtr& input_indices_ptr,
                                                  const PointCloudConstPtr& search_cloud_ptr)
  {
    PointCloudPtr source_cloud (new PointCloud);
    pcl17::ExtractIndices<PointType> extractor;
    extractor.setIndices(input_indices_ptr);
    extractor.setInputCloud(input_cloud_ptr);
    extractor.filter(*source_cloud);
    return getIndicesFromPointCloud(source_cloud, search_cloud_ptr);
  }

  pcl17::PointIndicesPtr getIndicesFromPointCloud(const PointCloudConstPtr& source_cloud,
                                                  const PointCloudConstPtr& search_cloud_ptr)
  {
    pcl17::PointIndicesPtr output_indices_ptr (new pcl17::PointIndices);
    pcl17::KdTreeFLANN<PointType> kdtreeNN;
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    kdtreeNN.setInputCloud(search_cloud_ptr);
    for(uint j = 0; j < source_cloud->points.size(); j++)
    {
      kdtreeNN.nearestKSearch(source_cloud->points[j], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
      output_indices_ptr->indices.push_back(pointIdxNKNSearch[0]);
    }
    return output_indices_ptr;
  }

  // sensor_msg::Image -> cv::Mat
  cv::Mat convertSensorMsgToCV (const sensor_msgs::Image& ros_image)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy (ros_image, ros_image.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR ("cv_bridge exception: %s", e.what ());
    }
    return cv_ptr->image;
  }

  // Pointcloud_2 -> pcl pointcloud
  PointCloudPtr convertSensorMsgPointCloudToPCL (sensor_msgs::PointCloud2& ros_pointcloud)
  {
    PointCloudPtr pointcloud_ptr (new PointCloud);
    pcl17::fromROSMsg (ros_pointcloud, *pointcloud_ptr);
    return pointcloud_ptr;
  }

  PointCloudPtr downsampleCloud(const PointCloudConstPtr input_cloud_ptr, const float leaf_size)
  {
    pcl17::VoxelGrid<PointType> sor;
    PointCloudPtr output_cloud_ptr (new PointCloud);
    sor.setInputCloud (input_cloud_ptr);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter (*output_cloud_ptr);
    return output_cloud_ptr;
  }

  void filterCloud (const PointCloudConstPtr input_cloud_ptr, const Eigen::Vector4f& min_point,
      const Eigen::Vector4f& max_point, const PointCloudPtr output_cloud_ptr)
  {
    // apply filter
    pcl17::CropBox<PointType> box_filter;
    box_filter.setInputCloud (input_cloud_ptr);
    box_filter.setMin (min_point);
    box_filter.setMax (max_point);
    box_filter.filter (*output_cloud_ptr);

  }

  void moveModelToOrigin (const PointCloudConstPtr cloud_input_ptr,
      const PointCloudPtr cloud_output_ptr,
      Eigen::Matrix4f& transform_output)
  {
    //find min in x,y and z.  Move this to Origin
    float min_x, min_y, min_z;
    min_x = min_y = min_z = std::numeric_limits<float>::max ();
    for (std::vector<PointType, Eigen::aligned_allocator<PointType> >::const_iterator itr =
        cloud_input_ptr->points.begin ();
        itr != cloud_input_ptr->points.end (); itr++)
    {
      if (itr->x < min_x)
        min_x = itr->x;
      if (itr->y < min_y)
        min_y = itr->y;
      if (itr->z < min_z)
        min_z = itr->z;
    }
    transform_output = Eigen::Matrix4f::Identity (4, 4);
    transform_output (0, 3) = -min_x;
    transform_output (1, 3) = -min_y;
    transform_output (2, 3) = -min_z;
    transformPointCloud (*cloud_input_ptr, *cloud_output_ptr, transform_output);
  }

}
