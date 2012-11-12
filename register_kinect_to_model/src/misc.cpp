#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Transform.h>
#include "tf_conversions/tf_eigen.h"


cv::Mat convertSensorMsgToCV(const sensor_msgs::Image& ros_image)
{

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(ros_image, ros_image.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return cv_ptr->image;
}


Eigen::Matrix4f convertTFToMatrix4f(const geometry_msgs::Transform& geometry_tf)
{
  tf::Transform tf_transform;
  tf::transformMsgToTF(geometry_tf,tf_transform);
  Eigen::Affine3d affine;
  tf::TransformTFToEigen(tf_transform, affine);
  Eigen::Matrix4f eigen_mat;
  for(int row = 0; row < 3; row++) for (int col = 0; col < 3; col++)
    eigen_mat(row,col) = affine.rotation()(row,col);
  for(int row = 0; row < 3; row++)
    eigen_mat(row,3) = affine.translation()(row);
  eigen_mat(3,3)=1;

  ROS_INFO_STREAM("geo msg \n "<<  geometry_tf);
  ROS_INFO_STREAM("mat \n " <<  eigen_mat);


//  geometry_msgs::Transform transform_msg;
//  Eigen::Matrix4d md(eigen_mat.cast<double>());
//  Eigen::Affine3d affine(md);
//  tf::Transform transform;
//  tf::TransformEigenToTF(affine, transform);
//  tf::transformTFToMsg(transform, transform_msg);
  return eigen_mat;
}
