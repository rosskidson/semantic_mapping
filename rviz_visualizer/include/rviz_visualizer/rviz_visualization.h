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
    void visualizeCloud (const sensor_msgs::PointCloud2& pointcloud_msg);
    void visualizeCloud (std::vector<PointCloudConstPtr>& cloud_ptr_vec);
    void visualizeCloud (PointCloudConstPtr cloud_ptr);
    void visualizeCloud (PointCloudConstPtr cloud_ptr, pcl17::PointIndicesConstPtr& cloud_indices_ptr);
    void visualizeCloud (PointCloudConstPtr cloud_ptr, std::vector<pcl17::PointIndicesConstPtr>& cloud_indices_ptrs);
    void visualizeCloudNormals (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr);
    void visualizeImage(const sensor_msgs::Image& image_msg);
    PointCloudConstPtr downsampleCloud (PointCloudConstPtr input);
    void spinOnce();

  private:
    double vox_grid_size_;

};

#endif /* RVIZ_VISUALIZATION_H_ */
