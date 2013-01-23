/*
 * visualization_base.h
 *
 *  Created on: Nov 10, 2012
 *      Author: Ross Kidson
 */

#ifndef VISUALIZATION_BASE_H_
#define VISUALIZATION_BASE_H_

// ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "pcl_typedefs/pcl_typedefs.h"

class VisualizationBase
{
  public:
    VisualizationBase ();
    virtual ~VisualizationBase ();

    //returns an id of the cloud. neccessary for interaction
    // rgb is for the colour of the cloud in the visualizer
    virtual int addCloudToVisualizer (PointCloudConstPtr cloud_ptr,
                                      double red=255,
                                      double green=255,
                                      double blue=255)=0;
    virtual void addNormalsToVisualizer (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr) =0;
    virtual void visualizeImage(const sensor_msgs::Image& image_msg) =0;
    virtual void removeAllClouds()=0;

    int visualizeCloud (PointCloudConstPtr cloud_ptr);
    int visualizeCloud (const sensor_msgs::PointCloud2& pointcloud_msg);
    int visualizeCloud (PointCloudConstPtr cloud_ptr, pcl17::PointIndicesConstPtr& cloud_indices_ptr);
    void visualizeCloudNormals (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr);
    PointCloudConstPtr downsampleCloud (PointCloudConstPtr input);

  protected:
    double vox_grid_size_;

};

#endif /* VISUALIZATION_BASE_H_ */
