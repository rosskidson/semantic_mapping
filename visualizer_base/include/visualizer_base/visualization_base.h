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

    virtual void visualizeCloud (PointCloudConstPtr cloud_ptr)=0;
    virtual void visualizeCloudNormals (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr) =0;
    virtual void visualizeImage(const sensor_msgs::Image& image_msg) =0;

    void visualizeCloud (const sensor_msgs::PointCloud2& pointcloud_msg);
    void visualizeCloud (std::vector<PointCloudConstPtr>& cloud_ptr_vec);
    void visualizeCloud (PointCloudConstPtr cloud_ptr, pcl17::PointIndicesConstPtr& cloud_indices_ptr);
    void visualizeCloud (PointCloudConstPtr cloud_ptr, std::vector<pcl17::PointIndicesConstPtr>& cloud_indices_ptrs);
    PointCloudConstPtr downsampleCloud (PointCloudConstPtr input);

  private:
    double vox_grid_size_;

};

#endif /* VISUALIZATION_BASE_H_ */
