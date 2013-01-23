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
                                      double red,
                                      double green,
                                      double blue)=0;
    virtual void addNormalsToVisualizer (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr) =0;
    virtual void visualizeImage(const sensor_msgs::Image& image_msg) =0;
    virtual void removeAllClouds()=0;

    int addCloudToVisualizer (PointCloudConstPtr cloud_ptr);
    int addCloudToVisualizer (const sensor_msgs::PointCloud2& pointcloud_msg);
    int addCloudToVisualizer (PointCloudConstPtr cloud_ptr, pcl17::PointIndicesConstPtr cloud_indices_ptr);
    void addCloudsToVisualizer(PointCloudConstPtr cloud_ptr, const std::vector<pcl17::PointIndicesConstPtr>& indices_ptrs);
    PointCloudConstPtr downsampleCloud (PointCloudConstPtr input);

  protected:
    double vox_grid_size_;

};

#endif /* VISUALIZATION_BASE_H_ */
