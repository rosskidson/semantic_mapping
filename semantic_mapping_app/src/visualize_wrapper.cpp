/*
 * wisualize_wrapper.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include "../include/semantic_mapping_app/visualize_wrapper.h"

VisualizeWrapper::VisualizeWrapper () :
  viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"))
{
  // TODO Auto-generated constructor stub

}

VisualizeWrapper::~VisualizeWrapper ()
{
  // TODO Auto-generated destructor stub
}

void VisualizeWrapper::visualizeCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr)
{
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
