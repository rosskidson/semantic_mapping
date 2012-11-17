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


Visualization::Visualization ()
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

void Visualization::visualizeCloud (PointCloudPtr cloud_ptr)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, "imported cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
      "imported cloud");
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
