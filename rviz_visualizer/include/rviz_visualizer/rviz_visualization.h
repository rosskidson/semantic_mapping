/*
 * rviz_visualization.h
 *
 *  Created on: Dec 22, 2012
 *      Author: Ross Kidson
 */

#ifndef RVIZ_VISUALIZATION_H_
#define RVIZ_VISUALIZATION_H_

#include <visualizer_base/visualization_base.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

#include "pcl_typedefs/pcl_typedefs.h"

class RVizVisualization : public VisualizationBase
{
  public:
    using VisualizationBase::addCloudToVisualizer;
    RVizVisualization ();
    virtual ~RVizVisualization ();
    virtual int addCloudToVisualizer (PointCloudConstPtr cloud_ptr,
                                      double red,
                                      double green,
                                      double blue);
    virtual void addNormalsToVisualizer (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr);
    virtual void visualizeImage(const sensor_msgs::Image& image_msg);
    virtual void removeAllClouds();
    virtual void spinOnce();

    void makeContextMenu();

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  private:
    static int cloud_counter_;
    interactive_markers::InteractiveMarkerServer interactive_marker_server_objects_;
    interactive_markers::MenuHandler menu_handler_;
    ros::NodeHandle nh_;
    ros::Publisher message_publisher_;

};

#endif /* RVIZ_VISUALIZATION_H_ */
