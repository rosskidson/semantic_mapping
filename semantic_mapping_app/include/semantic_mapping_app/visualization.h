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

class Visualization
{
  public:
    Visualization ();
    virtual ~Visualization ();
    void visualizeCloud (const sensor_msgs::PointCloud2& pointcloud_msg);
    void visualizeImage(const sensor_msgs::Image& image_msg);

};

#endif /* VISUALIZATION_H_ */
