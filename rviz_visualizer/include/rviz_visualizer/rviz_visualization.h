/*
 * rviz_visualization.h
 *
 *  Created on: Dec 22, 2012
 *      Author: Ross Kidson
 */

#ifndef RVIZ_VISUALIZATION_H_
#define RVIZ_VISUALIZATION_H_

// ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "pcl_typedefs/pcl_typedefs.h"

class RVizVisualization
{
  public:
    Visualization ();
    virtual ~Visualization ();
    virtual void visualizeClouds (std::vector<PointCloudConstPtr>& cloud_ptr_vec);
    virtual void visualizeCloudNormals (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr);
    virtual void visualizeImage(const sensor_msgs::Image& image_msg);

};

#endif /* RVIZ_VISUALIZATION_H_ */
