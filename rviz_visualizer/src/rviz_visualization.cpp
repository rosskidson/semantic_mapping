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
#include <std_msgs/String.h>
#include "qtgui/inputDialog.h"

//pcl
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

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

static const float POINT_SIZE = 0.005;

int RVizVisualization::cloud_counter_ = 0;

RVizVisualization::RVizVisualization ():
  nh_(),
  interactive_marker_server_objects_("semantic_mapping"),
  menu_handler_(),
  plane_counter_(0)
{
  this->makeContextMenu();
  message_publisher_ = nh_.advertise<std_msgs::String>("ui_popup", 1000);
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
  marker.scale.x = POINT_SIZE;
  marker.scale.y = POINT_SIZE;
  marker.scale.z = POINT_SIZE;

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
  marker_name << "Cloud_" << cloud_counter_++;
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
//  menu_handler_.insert( "Identify", boost::bind(&RVizVisualization::processFeedback, this, _1));
  menu_handler_.insert( "Rename", boost::bind(&RVizVisualization::processFeedback, this, _1));
//  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
//  menu_handler_.insert( sub_menu_handle, "First Entry", boost::bind(&RVizVisualization::processFeedback, this, _1));
//  menu_handler_.insert( sub_menu_handle, "Second Entry", boost::bind(&RVizVisualization::processFeedback, this, _1));
}


void RVizVisualization::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  ros::ServiceClient client;
  qtgui::inputDialog get_user_input_srv;
  visualization_msgs::InteractiveMarker int_marker;
//  s << "Feedback from marker '" << feedback->marker_name << "' "
//      << " / control '" << feedback->control_name << "'";
  s << feedback->marker_name;

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  std_msgs::String msg;
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
//      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
//      msg.data = s.str();
//      message_publisher_.publish(msg);
        client = nh_.serviceClient<qtgui::inputDialog> ("ui_dialog_service");
        get_user_input_srv.request.input.data = feedback->marker_name;
        if (!client.call (get_user_input_srv))
          ROS_INFO("service failed");
        interactive_marker_server_objects_.get(feedback->marker_name, int_marker);
        interactive_marker_server_objects_.erase(feedback->marker_name);
        int_marker.name = get_user_input_srv.response.output.data.c_str();
        interactive_marker_server_objects_.insert(int_marker);
        interactive_marker_server_objects_.setCallback(int_marker.name, boost::bind(&RVizVisualization::processFeedback, this, _1));
        interactive_marker_server_objects_.applyChanges();
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_DEBUG_STREAM( s.str() << ": pose changed"
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
      msg.data = s.str();
      //message_publisher_.publish(msg);
      break;
  }

  interactive_marker_server_objects_.applyChanges();
}

visualization_msgs::Marker RVizVisualization::makeBox( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

/*
 * The interactive marker needs to be rotated such that it moves on the plane
 * Without rotation the marker will move on the y/z plane (normal vector [1,0,0])
 * Therefore the rotation to be calculated is from [1,0,0] to the normal of the plane equation
 */

geometry_msgs::Quaternion RVizVisualization::convertModelCoefficientsToRotation(const pcl::ModelCoefficients::ConstPtr& coefficients_ptr)
{
    geometry_msgs::Quaternion rotation;
  double F = acos(coefficients_ptr->values[0])/2;
  rotation.w = cos(F);
  rotation.x = 0;
  rotation.y = -sin(F)*coefficients_ptr->values[2];
  rotation.z = sin(F)*coefficients_ptr->values[1];
    return rotation;
}

visualization_msgs::InteractiveMarker RVizVisualization::makeMarkerFromCoefficients(const pcl::ModelCoefficients::ConstPtr& coefficients,
                                                                                    const PointType& position,
                                                                                    const std::string& name)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.pose.position.x = position.x;
  int_marker.pose.position.y = position.y;
  int_marker.pose.position.z = position.z;
  int_marker.scale = 0.1;

  int_marker.name = name;
  int_marker.description = "complete plane marker";

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation = convertModelCoefficientsToRotation(coefficients);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);
    return int_marker;
}

void RVizVisualization::completePlane(const pcl::ModelCoefficients::ConstPtr &coefficients, const PointType& first_point, const PointType& last_point)
{
    plane_counter_++;
    std::stringstream plane_name_1, plane_name_2;
    plane_name_1 << "plane_marker_" << plane_counter_ << "_1";
    plane_name_2 << "plane_marker_" << plane_counter_ << "_2";

  interactive_marker_server_objects_.insert(makeMarkerFromCoefficients(coefficients, first_point, plane_name_1.str()));
  interactive_marker_server_objects_.insert(makeMarkerFromCoefficients(coefficients, last_point, plane_name_2.str()));
  interactive_marker_server_objects_.setCallback(plane_name_1.str(), boost::bind(&RVizVisualization::processFeedback, this, _1));
  interactive_marker_server_objects_.setCallback(plane_name_2.str(), boost::bind(&RVizVisualization::processFeedback, this, _1));
  interactive_marker_server_objects_.applyChanges();
}
