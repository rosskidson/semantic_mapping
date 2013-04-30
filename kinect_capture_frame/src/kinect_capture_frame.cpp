/*
 * main.cpp
 *
 * take kinect snapshot; saves pointcloud and iamge
 *
 *  Created on: Oct 30, 2012
 *      Author: rosskidson
 */

#include <ros/ros.h>

//image stuff
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//cloud stuff
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
// for string concat.
#include <sstream>
#include <iostream>

static const std::string cloud_topic = "/camera/depth_registered/points";
static const std::string image_topic = "/camera/rgb/image_color";

//sensor_msgs::ImageConstPtr ros_image_ptr;
//sensor_msgs::PointCloud2ConstPtr ros_pointcloud_ptr;

void takePointCloudSnapshot (const int image_no)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pointcloud_ptr (new (pcl::PointCloud<pcl::PointXYZRGB>));
  sensor_msgs::PointCloud2ConstPtr ros_pointcloud_ptr =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2> (cloud_topic, ros::Duration (5.0));
  if(!ros_pointcloud_ptr)
  {
    ROS_WARN_STREAM("No pointcloud message recieved");
    return;
  }
  pcl::fromROSMsg (*ros_pointcloud_ptr, *pcl_pointcloud_ptr);
  pcl::PCDWriter writer;
  std::stringstream filename;
  filename << "pointcloud_" << image_no << ".pcd";
  writer.write (filename.str (), *pcl_pointcloud_ptr);
  ROS_INFO_STREAM("wrote file " <<  filename.str() << " to file");
}

void takeImageSnapshot (const int image_no)
{
  sensor_msgs::ImageConstPtr ros_image_ptr = ros::topic::waitForMessage<sensor_msgs::Image> (
      image_topic, ros::Duration (5.0));
  if(!ros_image_ptr)
    {
      ROS_WARN_STREAM("No image message recieved");
      return;
    }
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy (ros_image_ptr, ros_image_ptr->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::stringstream filename;
  filename << "image_" << image_no << ".png";
  cv::imwrite (filename.str(), cv_ptr->image);
  ROS_INFO_STREAM("wrote file " <<  filename.str() << " to file");
}

//void imageCallback(const sensor_msgs::ImageConstPtr msg)
//{
//  ros_image_ptr = msg;
//}
//
//void cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)
//{
//  ros_pointcloud_ptr = msg;
//}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "mesh_io");
  ros::NodeHandle nh_;
  //image_subscriber_ = nh_.subscribe (image_topic, 1000, imageCallback);
  //pointcloud_subscriber_ = nh_.subscribe (cloud_topic, 1000, cloudCallback);

  ROS_INFO_STREAM("listening for pointclouds on " << cloud_topic);
  ROS_INFO_STREAM("listening for images on " << image_topic);
  int i (0);
  while (ros::ok ())
  {
    ROS_INFO("Press any key to take a screenshot");
    std::cin.get();
    ROS_INFO("Taking snapshot....");
    ros::spinOnce ();
    takePointCloudSnapshot (i);
    takeImageSnapshot (i);
    i++;
  }

  return 0;
}
