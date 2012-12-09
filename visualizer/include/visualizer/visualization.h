/*
 * visualization.h
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

// ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "pcl_typedefs/pcl_typedefs.h"

class Visualization
{
  public:
    Visualization ();
    virtual ~Visualization ();
    void visualizeCloud (const sensor_msgs::PointCloud2& pointcloud_msg);
    void visualizeCloud (std::vector<PointCloudConstPtr>& cloud_ptr_vec);
    void visualizeCloud (PointCloudConstPtr cloud_ptr);
    void visualizeImage(const sensor_msgs::Image& image_msg);
    PointCloudConstPtr downsampleCloud (PointCloudConstPtr input);
    void spinOnce();

  private:
    double vox_grid_size_;

};

#endif /* VISUALIZATION_H_ */
