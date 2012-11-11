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

#include "semantic_mapping_app/visualization.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");
  ros::NodeHandle nh;
  Visualization visualizer;

  ros::ServiceClient client;

  client = nh.serviceClient<mesh_io::loadModelFromFile> ("/mesh_io/load_model_from_file");
  mesh_io::loadModelFromFile load_model;
  load_model.request.filename = "/work/kidson/meshes/cabinet_scan_2/mesh_1.ply";
  ROS_INFO("Importing mesh to pointcloud....");
  if (!client.call (load_model))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  client = nh.serviceClient<mesh_io::loadImagesFromDir> ("/mesh_io/load_images_from_dir");
  mesh_io::loadImagesFromDir load_images;
  load_images.request.directory_name = "/work/kidson/meshes/cabinet_scan_2/KinFuSnapshots";
  ROS_INFO("loading registration screenshots...");
  if (!client.call (load_images))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  client = nh.serviceClient<mesh_io::loadTransformationsFromDir> ("/mesh_io/load_transforms_from_dir");
  mesh_io::loadTransformationsFromDir load_transforms;
  load_transforms.request.directory_name = "/work/kidson/meshes/cabinet_scan_2/KinFuSnapshots";
  ROS_INFO("loading registration transformations...");
  if (!client.call (load_transforms))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  client = nh.serviceClient<kinect_capture_frame::kinectSnapshot> ("kinect_snapshot_service");
  kinect_capture_frame::kinectSnapshot get_kinect_frame;
  ROS_INFO("getting snapshot from kinect");
  if(!client.call(get_kinect_frame))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  client = nh.serviceClient<register_kinect_to_model::registerKinectToModel> ("/kinect_registration/register_kinect_to_model");
  register_kinect_to_model::registerKinectToModel register_kinect_srv;
  register_kinect_srv.request.kinect_cloud = get_kinect_frame.response.pointcloud;
  register_kinect_srv.request.kinect_image = get_kinect_frame.response.image;
  register_kinect_srv.request.registration_images = load_images.response.images;
  register_kinect_srv.request.registration_transforms = load_transforms.response.transformations;
  register_kinect_srv.request.model = load_model.response.pointcloud;
  ROS_INFO("Registering Kinect to Model.... ");
  if (!client.call (register_kinect_srv))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  //visualizer.visualizeCloud(load_model.response.pointcloud);

  return 0;
}
