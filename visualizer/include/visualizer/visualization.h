/*
 * visualization.h
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include "visualizer_base/visualization_base.h"

// ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "pcl_typedefs/pcl_typedefs.h"

class Visualization : public VisualizationBase
{
  public:
    using VisualizationBase::visualizeClouds;

    Visualization ();
    virtual ~Visualization ();
    virtual void visualizeClouds (std::vector<PointCloudConstPtr>& cloud_ptr_vec);
    void virtual visualizeCloudNormals (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr);
    void virtual visualizeImage(const sensor_msgs::Image& image_msg);
    void spinOnce();


};

#endif /* VISUALIZATION_H_ */
