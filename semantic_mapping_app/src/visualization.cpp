/*
 * visualization.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include "../include/semantic_mapping_app/visualization.h"

// ros
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>
//vtk
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// opencv -> ROS -> opencv
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// opencv
#include <opencv2/highgui/highgui.hpp>

#include <iostream>


Visualization::Visualization (): vox_grid_size_(0.02)
{
  // TODO Auto-generated constructor stub

}

Visualization::~Visualization ()
{
  // TODO Auto-generated destructor stub
}

void Visualization::visualizeCloud (const sensor_msgs::PointCloud2& pointcloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pointcloud_msg,*cloud_ptr);

  visualizeCloud(cloud_ptr);
}

void Visualization::visualizeCloud (PointCloudConstPtr cloud_ptr)
{
  std::vector<PointCloudConstPtr> cloud_ptr_vec;
  cloud_ptr_vec.push_back(cloud_ptr);
  visualizeCloud(cloud_ptr_vec);
}

void Visualization::visualizeCloud (std::vector<PointCloudConstPtr>& cloud_ptr_vec)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  for(std::vector<PointCloudConstPtr>::iterator itr=cloud_ptr_vec.begin(); itr != cloud_ptr_vec.end(); itr++)
  {
    int cloud_num = itr - cloud_ptr_vec.begin();
    std::stringstream cloud_name;
    cloud_name << "cloud "<< itr - cloud_ptr_vec.begin();

    // downsample cloud if needed
    PointCloudConstPtr downsampled_ptr;
    if(vox_grid_size_ > 0.0)
      downsampled_ptr = downsampleCloud(*itr);
    else
      downsampled_ptr = *itr;

    // different colours for different clouds
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
      single_color(downsampled_ptr, 127*(cloud_num % 3), 127*((cloud_num+1) % 3), 127*((cloud_num+2) % 3));

    //add the cloud
    viewer->addPointCloud<PointType> (downsampled_ptr, single_color, cloud_name.str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
      cloud_name.str());
  }
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void Visualization::visualizeImage(const sensor_msgs::Image& image_msg)
{
  sensor_msgs::ImagePtr msg_ptr (new sensor_msgs::Image(image_msg));
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy (msg_ptr, msg_ptr->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  cv::imshow("abc", cv_ptr->image);
  cv::waitKey(0);
}

PointCloudConstPtr Visualization::downsampleCloud (PointCloudConstPtr input)
{
  const double voxel_size = vox_grid_size_;
  PointCloudPtr cloud_filtered (new PointCloud);
  pcl::VoxelGrid<PointType> downsampler;
  downsampler.setInputCloud (input);
  downsampler.setLeafSize (voxel_size, voxel_size, voxel_size);
  downsampler.filter (*cloud_filtered);
  return cloud_filtered;
}
