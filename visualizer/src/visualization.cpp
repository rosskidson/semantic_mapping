/*
 * visualization.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include "visualizer/visualization.h"

// ros
#include <ros/console.h>

//pcl
#include <pcl17/point_types.h>

//vtk
#include <boost/thread/thread.hpp>
#include <pcl17/common/common_headers.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <pcl17/console/parse.h>

// opencv -> ROS -> opencv
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// opencv
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

// singleton instance of viewer
static boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer_;
int Visualization::cloud_counter_ = 0;

Visualization::Visualization ()
{
  if(!viewer_)    // instantiate viewer
  {
    viewer_.reset(new pcl17::visualization::PCLVisualizer ("3D Viewer"));
    viewer_->addCoordinateSystem (1.0);
    viewer_->initCameraParameters ();
    viewer_->setBackgroundColor (0, 0, 0);
  }
}

Visualization::~Visualization ()
{
  // TODO Auto-generated destructor stub
}

void Visualization::removeAllClouds()
{
  viewer_->removeAllPointClouds();
}

int Visualization::addCloudToVisualizer(PointCloudConstPtr cloud_ptr, double red, double green, double blue)
{
  pcl17::visualization::PointCloudColorHandlerCustom<PointType>
      single_color(cloud_ptr, red, green, blue);

  std::stringstream cloud_name;
  cloud_name << "cloud "<< cloud_counter_++;

  viewer_->addPointCloud<PointType> (cloud_ptr, single_color, cloud_name.str());
  viewer_->setPointCloudRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 1, cloud_name.str());
  this->spinOnce();
  return cloud_counter_;
}


void Visualization::addNormalsToVisualizer (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr)
{
  viewer_->addPointCloudNormals<PointType, PointNormal>(cloud_ptr, cloud_normals_ptr, 20);
  viewer_->setPointCloudRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 1);
  this->spinOnce();
}

void Visualization::spinOnce()
{
  viewer_->spinOnce (100);
  //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
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

