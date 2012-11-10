#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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
