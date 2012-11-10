/*
 * main.cpp
 *
 * take kinect snapshot; saves pointcloud and iamge
 *
 *  Created on: Oct 30, 2012
 *      Author: rosskidson
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcl_tools/kinectSnapshot.h"

static const std::string cloud_topic = "/camera/depth_registered/points";
static const std::string image_topic = "/camera/rgb/image_color";

sensor_msgs::ImageConstPtr ros_image_ptr;
sensor_msgs::PointCloud2ConstPtr ros_pointcloud_ptr;

void imageCallback(const sensor_msgs::ImageConstPtr msg)
{
  ros_image_ptr = msg;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
  ros_pointcloud_ptr = msg;
}

bool getSnapshot(pcl_tools::kinectSnapshot::Request& req, pcl_tools::kinectSnapshot::Response& res)
{
  if(!ros_image_ptr || !ros_pointcloud_ptr)
  {
    ROS_WARN("No data recieved from kinect");
    return false;
  }
  res.image = *ros_image_ptr;
  res.pointcloud = *ros_pointcloud_ptr;
  return true;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "kinect_snapshot_service");
  ros::NodeHandle nh;
  ros::Subscriber image_subscriber_ = nh.subscribe (image_topic, 1000, imageCallback);
  ros::Subscriber pointcloud_subscriber_ = nh.subscribe (cloud_topic, 1000, cloudCallback);
  ros::ServiceServer service = nh.advertiseService("kinect_snapshot_service", getSnapshot);

  ROS_INFO_STREAM("listening for pointclouds on " << cloud_topic);
  ROS_INFO_STREAM("listening for images on " << image_topic);
  ros::spin();

  return 0;
}
