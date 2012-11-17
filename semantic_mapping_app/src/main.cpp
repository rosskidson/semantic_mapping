/*
 * main.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include <ros/ros.h>
#include "pcl_typedefs/pcl_typedefs.h"

// get a kinect frame service
#include "kinect_capture_frame/kinectSnapshot.h"

#include "mesh_io/mesh_io.h"

#include "box_filter/box_filter.h"

#include "semantic_mapping_app/visualization.h"

#include "semantic_mapping_app/parameter_server.h"

#include <Eigen/Core>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");
  ros::NodeHandle nh;
  Visualization visualizer;

  // import mesh
  MeshIO io_obj;

  ROS_INFO("Importing mesh to pointcloud model...");
  PointCloudConstPtr raw_import;
  raw_import = io_obj.loadMeshFromFile (ParameterServer::instance ()->get<std::string> (
      "mesh_input_filename"));

  ROS_INFO("Applying boxfilter to cloud...");
  PointCloudPtr model (new PointCloud);
  Eigen::Vector4f min_point (1.0, 1.0, 0.6, 1);
  Eigen::Vector4f max_point (2.0, 2.7, 1.2, 1);
  box_filter::filterCloud (raw_import, min_point, max_point, model);

  visualizer.visualizeCloud(model);

  ros::ServiceClient client;
  client = nh.serviceClient<kinect_capture_frame::kinectSnapshot> ("kinect_snapshot_service");
  kinect_capture_frame::kinectSnapshot get_kinect_frame_srv;
  ROS_INFO("getting snapshot from kinect");
  if (!client.call (get_kinect_frame_srv))
  {
    ROS_INFO("A service failed");
    return 1;
  }



  return 0;
}
