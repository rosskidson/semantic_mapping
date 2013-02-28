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
#include <pcl17/ModelCoefficients.h>

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

    void completePlane(const pcl17::ModelCoefficients::ConstPtr &coefficients, const PointType& first_point, const PointType& last_point);

    void createNewPlane();

    visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg,
          const double x_scale, const double y_scale, const double z_scale );

    visualization_msgs::InteractiveMarker makeMarkerFromCoefficients(const pcl17::ModelCoefficients::ConstPtr& coefficients, const PointType &position, const std::string &name);

    void makeROImarkers();

  private:
    static int cloud_counter_;
    int plane_counter_;
    interactive_markers::InteractiveMarkerServer interactive_marker_server_objects_;
    interactive_markers::MenuHandler menu_handler_;
    ros::NodeHandle nh_;
    ros::Publisher message_publisher_;

    int hax;

};

#endif /* RVIZ_VISUALIZATION_H_ */
