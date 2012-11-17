/*
 * main.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include <ros/ros.h>

// get a kinect frame service
#include "kinect_capture_frame/kinectSnapshot.h"

#include "mesh_io/mesh_converter.h"

#include "semantic_mapping_app/visualization.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");
  ros::NodeHandle nh;
  Visualization visualizer;

  // import mesh

  ros::ServiceClient client;
  client = nh.serviceClient<kinect_capture_frame::kinectSnapshot> ("kinect_snapshot_service");
  kinect_capture_frame::kinectSnapshot get_kinect_frame_srv;
  ROS_INFO("getting snapshot from kinect");
  if(!client.call(get_kinect_frame_srv))
  {
    ROS_INFO("A service failed");
    return 1;
  }


//  min_point.x = 1.0;
//  min_point.y = 1.0;
//  min_point.z = 0.6;
//  max_point.x = 2.0;
//  max_point.y = 2.7;
//  max_point.z = 1.2;


  //visualizer.visualizeCloud(load_model.response.pointcloud);

  return 0;
}
