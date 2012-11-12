/*
 * main.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include <ros/ros.h>

// io services
#include "mesh_io/loadImagesFromDir.h"
#include "mesh_io/loadModelFromFile.h"
#include "mesh_io/loadPointcloudsFromDir.h"
#include "mesh_io/loadTransformationsFromDir.h"
// register kinect service
#include "register_kinect_to_model/registerKinectToModel.h"
// get a kinect frame service
#include "kinect_capture_frame/kinectSnapshot.h"
// boxfilter service
#include "box_filter/boxFilter.h"

#include "semantic_mapping_app/visualization.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");
  ros::NodeHandle nh;
  Visualization visualizer;

  ros::ServiceClient client;

  client = nh.serviceClient<mesh_io::loadModelFromFile> ("/mesh_io/load_model_from_file");
  mesh_io::loadModelFromFile load_model_srv;
  ////media/burg/work/meshes/chair_2/nontextured/chair2.ply
  load_model_srv.request.filename = "/work/kidson/meshes/cabinet_scan_2/mesh_1.ply";
  ROS_INFO("Importing mesh to pointcloud....");
  if (!client.call (load_model_srv))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  client = nh.serviceClient<box_filter::boxFilter> ("/box_filter/box_filter");
  box_filter::boxFilter box_filter_srv;
  box_filter_srv.request.input_cloud = load_model_srv.response.pointcloud;
  geometry_msgs::Vector3 min_point, max_point;
  min_point.x = 1.0;
  min_point.y = 1.0;
  min_point.z = 0.6;
  max_point.x = 2.0;
  max_point.y = 2.7;
  max_point.z = 1.2;
  box_filter_srv.request.input_min_point = min_point;
  box_filter_srv.request.input_max_point = max_point;
  ROS_INFO("box filter operation....");
  if (!client.call (box_filter_srv))
  {
    ROS_INFO("A service failed");
    return 1;
  }
  visualizer.visualizeCloud(box_filter_srv.response.output);

  client = nh.serviceClient<mesh_io::loadImagesFromDir> ("/mesh_io/load_images_from_dir");
  mesh_io::loadImagesFromDir load_images_srv;
  load_images_srv.request.directory_name = "/work/kidson/meshes/cabinet_scan_2/KinFuSnapshots";
  ROS_INFO("loading registration screenshots...");
  if (!client.call (load_images_srv))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  client = nh.serviceClient<mesh_io::loadTransformationsFromDir> ("/mesh_io/load_transforms_from_dir");
  mesh_io::loadTransformationsFromDir load_transforms_srv;
  load_transforms_srv.request.directory_name = "/work/kidson/meshes/cabinet_scan_2/KinFuSnapshots";
  ROS_INFO("loading registration transformations...");
  if (!client.call (load_transforms_srv))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  client = nh.serviceClient<kinect_capture_frame::kinectSnapshot> ("kinect_snapshot_service");
  kinect_capture_frame::kinectSnapshot get_kinect_frame_srv;
  ROS_INFO("getting snapshot from kinect");
  if(!client.call(get_kinect_frame_srv))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  client = nh.serviceClient<register_kinect_to_model::registerKinectToModel> ("/kinect_registration/register_kinect_to_model");
  register_kinect_to_model::registerKinectToModel register_kinect_srv;
  register_kinect_srv.request.kinect_cloud = get_kinect_frame_srv.response.pointcloud;
  register_kinect_srv.request.kinect_image = get_kinect_frame_srv.response.image;
  register_kinect_srv.request.registration_images = load_images_srv.response.images;
  register_kinect_srv.request.registration_transforms = load_transforms_srv.response.transformations;
  register_kinect_srv.request.model = load_model_srv.response.pointcloud;
  ROS_INFO("Registering Kinect to Model.... ");
  if (!client.call (register_kinect_srv))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  //visualizer.visualizeCloud(load_model.response.pointcloud);

  return 0;
}
