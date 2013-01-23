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
    using VisualizationBase::addCloudToVisualizer;
    Visualization ();
    virtual ~Visualization ();
    virtual int addCloudToVisualizer (PointCloudConstPtr cloud_ptr,
                                      double red,
                                      double green,
                                      double blue);
    virtual void addNormalsToVisualizer (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr);
    virtual void visualizeImage(const sensor_msgs::Image& image_msg);
    virtual void removeAllClouds();

    void spinOnce();
  private:
    static int cloud_counter_;

};

#endif /* VISUALIZATION_H_ */
