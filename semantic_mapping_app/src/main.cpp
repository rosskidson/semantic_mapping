/*
 * main.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include <ros/ros.h>

// get a kinect frame service
#include "kinect_capture_frame/kinectSnapshot.h"

// includes from this stack
#include "pcl_typedefs/pcl_typedefs.h"
#include "mesh_io/mesh_io.h"
#include "box_filter/box_filter.h"
#include "register_kinect_to_model/kinect_registration.h"
#include "align_principle_axis/axis_alignment.h"
#include "segment_planes_interface/plane_segmentation.h"

#include <pluginlib/class_loader.h>

#include "semantic_mapping_app/visualization.h"
#include "semantic_mapping_app/parameter_server.h"

//convert sensor msgs
#include "message_conversions.cpp"

#include <Eigen/Core>

//#include <pcl17/features/normal_3d.h>
//#include <pcl17/filters/voxel_grid.h>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");
  ros::NodeHandle nh;
  Visualization visualizer;
  ParameterServer* ps = ParameterServer::instance ();
  std::vector<PointCloudConstPtr> vis_clouds;

  // import mesh
  MeshIO io_obj;

  ROS_INFO("Importing mesh to pointcloud model...");
  PointCloudPtr raw_import_ptr;
  raw_import_ptr = io_obj.loadMeshFromFile (ps->get<std::string> ("mesh_input_filename"));
  io_obj.savePointcloudToFile(raw_import_ptr, "raw_import.pcd");

  ROS_INFO("Performing principle axis alignment...");
  //align_principle_axis::FloorAxisAlignment axis_align;

  pluginlib::ClassLoader<align_principle_axis::AxisAlignment> loader_axis("align_principle_axis", "align_principle_axis::AxisAlignment");

  align_principle_axis::AxisAlignment* axis_align = NULL;

  try
  {
    axis_align = loader_axis.createClassInstance("align_principle_axis/FloorAxisAlignment");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  Eigen::Matrix4f guess, align_trafo, move_to_origin;
  guess = Eigen::Matrix4f::Zero(4,4);
  guess(0,0) = 1.0;
  guess(1,2) = 1.0;
  guess(2,1) = -1.0;
  guess(3,3) = 1.0;
  PointCloudPtr model_aligned_ptr (new PointCloud);
  axis_align->alignCloudPrincipleAxis(raw_import_ptr, guess, model_aligned_ptr, align_trafo);

  //visualizer.visualizeCloud(model_aligned_ptr);

  ROS_INFO("Applying boxfilter to cloud...");
  PointCloudPtr cabinet_cloud_ptr (new PointCloud);
  Eigen::Vector4f min_point (0.9, 0.8, -3.0, 1);
  Eigen::Vector4f max_point (1.8, 1.4, -1.3, 1);
  box_filter::filterCloud (model_aligned_ptr, min_point, max_point, cabinet_cloud_ptr);

  ROS_INFO("move model to origin...");
  PointCloudPtr cabinet_centered_cloud_ptr (new PointCloud);
  axis_align->moveModelToOrigin(cabinet_cloud_ptr, cabinet_centered_cloud_ptr, move_to_origin);
  visualizer.visualizeCloud(cabinet_centered_cloud_ptr);


  ROS_INFO("segment planes...");
  //align_principle_axis::FloorAxisAlignment axis_align;

  pluginlib::ClassLoader<segment_planes_interface::PlaneSegmentation> loader_planes("segment_planes_interface", "segment_planes_interface::PlaneSegmentation");

  segment_planes_interface::PlaneSegmentation* plane_segmenter = NULL;

  try
  {
    plane_segmenter = loader_planes.createClassInstance("segment_planes_region_grow_plugin/PlaneSegmentationRegionGrow");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  std::vector<PointCloudConstPtr> dfg;
  std::vector<pcl17::ModelCoefficients::ConstPtr> hij;

//  PointCloudPtr temp (new PointCloud);
//  double leaf_size = 0.01;
//  pcl17::VoxelGrid<PointType> sor;
//  sor.setInputCloud (cabinet_centered_cloud_ptr);
//  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
//  sor.filter (*temp);

  plane_segmenter->segmentPlanes(cabinet_centered_cloud_ptr, dfg,hij);
//
//  PointCloudNormalsPtr cloud_normals (new PointCloudNormals);
//  pcl17::NormalEstimation<PointType, PointNormal> ne;
//  //ne.setNumberOfThreads(4);
//  ne.setInputCloud (temp);
//
//  pcl17::search::KdTree<PointType>::Ptr normals_tree (new pcl17::search::KdTree<PointType>);
//  //ne.setKSearch(30);
//  ne.setRadiusSearch(0.1);
//  ne.setSearchMethod(normals_tree);
//  ne.compute (*cloud_normals);
//
//  std::vector<PointCloudConstPtr> dfg;
//  std::vector<pcl17::ModelCoefficients::ConstPtr> hij;
//
//  plane_segmenter->segmentPlanes(temp,cloud_normals, dfg,hij);

//  ros::ServiceClient client;
//  client = nh.serviceClient<kinect_capture_frame::kinectSnapshot> ("kinect_snapshot_service");
//  kinect_capture_frame::kinectSnapshot get_kinect_frame_srv;
//  ROS_INFO("getting snapshot from kinect");
//  if (!client.call (get_kinect_frame_srv))
//  {
//    ROS_INFO("kinect snapshot service failed");
//    return 1;
//  }
//  PointCloudPtr kinect_cloud_ptr = convertSensorMsgPointCloudToPCL(get_kinect_frame_srv.response.pointcloud);
//  cv::Mat kinect_image = convertSensorMsgToCV(get_kinect_frame_srv.response.image);
//
//  ROS_INFO("using kinect snapshot from file");
//  kinect_image = io_obj.loadImageFromFile("/work/kidson/meshes/cabinet_scan_3/frames_to_register/image_2.png");
//  kinect_cloud_ptr = io_obj.loadPointcloudFromFile("/work/kidson/meshes/cabinet_scan_3/frames_to_register/pointcloud_2.pcd");
//  io_obj.savePointcloudToFile(kinect_cloud_ptr, "kinect_cloud.pcd");
//  io_obj.savePointcloudToFile(model_cloud_ptr, "model_cloud.pcd");
//
//  ROS_INFO("Registering Kinect to Model...");
//  std::vector<cv::Mat> images;
//  std::vector<Eigen::Matrix4f> transformations;
//  io_obj.loadImagesFromDir(ps->get<std::string> ("mesh_registration_images_directory"),images);
//  io_obj.loadTransformationsFromDir(ps->get<std::string> ("mesh_registration_transformations_directory"),transformations);
//  KinectRegistration kinect_reg;
//  Eigen::Matrix4f dildos;
//  dildos = kinect_reg.registerKinectToModel(model_cloud_ptr,kinect_cloud_ptr,kinect_image,images,transformations);
//  ROS_INFO_STREAM("final trafo \n " << dildos);

  return 0;
}
