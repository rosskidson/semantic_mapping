/*
 * rviz_visualization.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: Ross Kidson
 */

#include "rviz_visualizer/rviz_visualization.h"

// ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/ros.h>
#include <ros/console.h>

//pcl
#include <pcl17/point_types.h>
#include <pcl17/ros/conversions.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/filters/extract_indices.h>

//visualization msgs
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

//interactive markers
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <QApplication>
#include <QMessageBox>
#include <QPushButton>
#include <QWidget>
#include <QObject>


//class MainWindow : public QWidget
//{
//    Q_OBJECT
//public slots:
//      void buttonPressed();
//};

//void MainWindow::buttonPressed()
//{
//    QMessageBox::information(0, QString("Information"), QString("You've pressed the button \"Press Me!\""), QMessageBox::Ok);
//}

int RVizVisualization::cloud_counter_ = 0;

RVizVisualization::RVizVisualization ():
  interactive_marker_server_objects_("semantic_mapping"),
  menu_handler_()
{
  this->makeContextMenu();


  QMessageBox msgBox;
  msgBox.setText("The document has been modified.");
  msgBox.exec();
  //return a.exec();
}

RVizVisualization::~RVizVisualization ()
{

}

/**
 * Add a pointcloud to the visualizer object
 * @param[in] cloud_ptr  Pointer to pointcloud
 * @param[in] red Specify colour for the cloud to visualize
 * @param[in] green Specify colour for the cloud to visualize
 * @param[in] blue Specify colour for the cloud to visualize
 * @return id of the cloud in the visualization obj.
 */
int RVizVisualization::addCloudToVisualizer (PointCloudConstPtr cloud_ptr, double red, double green,double blue)
{
  // create marker object
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::POINTS;
  geometry_msgs::Point p;
  for (size_t j = 0; j < cloud_ptr->points.size(); j++)
  {
    p.x = cloud_ptr->points.at(j).x;
    p.y = cloud_ptr->points.at(j).y;
    p.z = cloud_ptr->points.at(j).z;
    marker.points.push_back(p);
  }
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration(0);
  marker.scale.x = 0.003;
  marker.scale.y = 0.003;
  marker.scale.z = 0.003;

  // create control object
//  visualization_msgs::InteractiveMarkerControl control;
//  control.always_visible = true;
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
//  control.markers.push_back(marker);

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";
  control.markers.push_back( marker );

  // create point cloud marker
  std::stringstream marker_name;
  marker_name << "cloud" << cloud_counter_++;
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.name = marker_name.str();
  int_marker.description = "PointCloud";
  int_marker.controls.push_back(control);

  interactive_marker_server_objects_.insert(int_marker);
  interactive_marker_server_objects_.setCallback(int_marker.name, boost::bind(&RVizVisualization::processFeedback, this, _1));
  menu_handler_.apply( interactive_marker_server_objects_, int_marker.name );
  interactive_marker_server_objects_.applyChanges();

  return cloud_counter_;
}

Eigen::Quaternionf dir2quat(Eigen::Vector3f d)
{
//????
  return Eigen::Quaternionf();
}

/**
 * Add normals to the visualizer object
 * @param[in] cloud_ptr  Pointer to pointcloud
 * @param[in] cloud_normals_ptr  Pointer to normals of the pointcloud
 */
void RVizVisualization::addNormalsToVisualizer (PointCloudConstPtr cloud_ptr, PointCloudNormalsConstPtr cloud_normals_ptr)
{
  int normal_counter=0;

  // putting this many interactive markers in rviz is too slow
  for (size_t j = 0; j < cloud_ptr->points.size(); j+=100)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::ARROW;

    marker.pose.position.x = cloud_ptr->at(j).x;
    marker.pose.position.y = cloud_ptr->at(j).y;
    marker.pose.position.z = cloud_ptr->at(j).z;

    const PointNormal& normal = cloud_normals_ptr->points.at(j);
    Eigen::Quaternionf q = dir2quat(Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z));
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.lifetime = ros::Duration(0);
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    // create control object
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.markers.push_back(marker);

    // create point cloud marker
    std::stringstream marker_name;
    marker_name << "normal" << normal_counter++;
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/base_link";
    int_marker.name = marker_name.str();
    int_marker.description = "Normals";
    int_marker.controls.push_back(control);

//    interactive_marker_server_objects_.insert(int_marker);
  }
//  interactive_marker_server_objects_.applyChanges();
}

void RVizVisualization::visualizeImage(const sensor_msgs::Image& image_msg)
{

}

void RVizVisualization::removeAllClouds()
{
  interactive_marker_server_objects_.clear();
  interactive_marker_server_objects_.applyChanges();
}

void RVizVisualization::spinOnce()
{

}

/**
 * Creates menu items for context menu
 */
void RVizVisualization::makeContextMenu()
{
  menu_handler_.insert( "First Entry", boost::bind(&RVizVisualization::processFeedback, this, _1));
  menu_handler_.insert( "Second Entry", boost::bind(&RVizVisualization::processFeedback, this, _1));
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
  menu_handler_.insert( sub_menu_handle, "First Entry", boost::bind(&RVizVisualization::processFeedback, this, _1));
  menu_handler_.insert( sub_menu_handle, "Second Entry", boost::bind(&RVizVisualization::processFeedback, this, _1));
}


void RVizVisualization::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  interactive_marker_server_objects_.applyChanges();
}
