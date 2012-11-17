#include "cv_bridge/cv_bridge.h"
#include "pcl/ros/conversions.h"
#include "pcl_typedefs/pcl_typedefs.h"

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
  pcl::fromROSMsg (ros_pointcloud, *pointcloud_ptr);
  return pointcloud_ptr;
}
