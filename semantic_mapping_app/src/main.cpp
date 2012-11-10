/*
 * main.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */


#include <ros/ros.h>
#include "mesh_io/loadImagesFromDir.h"
#include "mesh_io/loadModelFromFile.h"
#include "mesh_io/loadPointcloudsFromDir.h"
#include "mesh_io/loadTransformationsFromDir.h"

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include "semantic_mapping_app/visualize_wrapper.h"

//io
#include <pcl/io/pcd_io.h>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");
  ros::NodeHandle nh;

  ros::ServiceClient client;

  //nh.serviceClient<mesh_io::loadImagesFromDir>("load_images_from_dir");
  //nh.serviceClient<mesh_io::loadTransformationsFromDir>("load_images_from_dir");
  //nh.serviceClient<mesh_io::loadModelFromFile>("load_images_from_dir");

  client = nh.serviceClient<mesh_io::loadModelFromFile>("/mesh_io/load_model_from_file");
  mesh_io::loadModelFromFile load_service;
  load_service.request.filename = "/work/kidson/meshes/cabinet_scan_2/mesh_1.ply";
  if (!client.call(load_service))
  {
    ROS_INFO("A service failed");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(load_service.response.pointcloud,*cloud_ptr);

  VisualizeWrapper visualizer;
  visualizer.visualizeCloud(cloud_ptr);

  return 0;
}
