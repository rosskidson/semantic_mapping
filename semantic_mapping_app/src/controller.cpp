/*
 * controller.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: ross kidson
 */

#include "semantic_mapping_app/controller.h"
#include "semantic_mapping_app/parameter_server.h"
#include "message_conversions.cpp"

// includes from this stack
#include "kinect_capture_frame/kinectSnapshot.h"
#include "pcl_typedefs/pcl_typedefs.h"
#include "mesh_io/mesh_io.h"
#include "box_filter/box_filter.h"
#include "register_kinect_to_model/kinect_registration.h"
#include "align_principle_axis/axis_alignment.h"
#include "segment_planes_interface/plane_segmentation.h"
#include "visualizer/visualization.h"

#include <ros/ros.h>
#include <pluginlib/class_loader.h>


//convert sensor msgs

#include <Eigen/Core>

typedef std::pair<std::string, PointCloudConstPtr> NamedPointCloudPtr;

Controller::Controller():
    nh_("~"), visualizer_()
{

}


Controller::~Controller()
{
    //destructor
}

void Controller::importScan()
{
  ROS_INFO("Importing mesh to pointcloud model...");
  PointCloudPtr raw_scan_pointcloud_ptr;
  raw_scan_pointcloud_ptr = io_obj_.loadMeshFromFile (ParameterServer::instance()->get<std::string>
                                            ("mesh_input_filename"));
  pointcloud_ptrs.insert(NamedPointCloudPtr("raw_scan",raw_scan_pointcloud_ptr));
  //visualizer_.visualizeCloud(pointcloud_ptrs["raw_scan"]);
}

void Controller::alignToPrincipleAxis()
{
  ROS_INFO("Performing principle axis alignment...");
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

  Eigen::Matrix4f guess;
  guess = Eigen::Matrix4f::Zero(4,4);
  guess(0,0) = 1.0;
  guess(1,2) = 1.0;
  guess(2,1) = -1.0;
  guess(3,3) = 1.0;
  PointCloudPtr aligned_scan_ptr (new PointCloud);
  axis_align->alignCloudPrincipleAxis(pointcloud_ptrs["raw_scan"], guess, aligned_scan_ptr, align_to_axis_);
  pointcloud_ptrs.insert(NamedPointCloudPtr("aligned_scan",aligned_scan_ptr));
  //visualizer_.visualizeCloud(pointcloud_ptrs["aligned_scan"]);
}

void Controller::extractROI()
{
  //TODO:: REMOVE.  this is here because of base class functionality in axisalign.  move to pcl_tools
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

  ROS_INFO("Applying boxfilter to cloud...");
  PointCloudPtr cabinet_cloud_ptr (new PointCloud);
  Eigen::Vector4f min_point (0.9, 0.8, -3.0, 1);
  Eigen::Vector4f max_point (1.85, 1.4, -1.2, 1);
  box_filter::filterCloud (pointcloud_ptrs["aligned_scan"], min_point, max_point, cabinet_cloud_ptr);

  ROS_INFO("move model to origin...");
  PointCloudPtr cabinet_centered_cloud_ptr (new PointCloud);
  axis_align->moveModelToOrigin(cabinet_cloud_ptr, cabinet_centered_cloud_ptr, move_model_to_origin_);
  pointcloud_ptrs.insert(NamedPointCloudPtr("model",cabinet_centered_cloud_ptr));
  visualizer_.visualizeCloud(pointcloud_ptrs["model"]);
}

void Controller::segmentPlanes()
{
  ROS_INFO("segment planes...");
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
  plane_segmenter->segmentPlanes(pointcloud_ptrs["model"], dfg,hij);
}

void Controller::registerKinectToModel()
{
  ros::ServiceClient client;
  client = nh_.serviceClient<kinect_capture_frame::kinectSnapshot> ("kinect_snapshot_service");
  kinect_capture_frame::kinectSnapshot get_kinect_frame_srv;
  ROS_INFO("getting snapshot from kinect");
  if (!client.call (get_kinect_frame_srv))
  {
    ROS_INFO("kinect snapshot service failed");
    return;
  }
  PointCloudPtr kinect_cloud_ptr = convertSensorMsgPointCloudToPCL(get_kinect_frame_srv.response.pointcloud);
  cv::Mat kinect_image = convertSensorMsgToCV(get_kinect_frame_srv.response.image);

  ROS_INFO("using kinect snapshot from file");
  kinect_image = io_obj_.loadImageFromFile("/work/kidson/meshes/cabinet_scan_3/frames_to_register/image_2.png");
  kinect_cloud_ptr = io_obj_.loadPointcloudFromFile("/work/kidson/meshes/cabinet_scan_3/frames_to_register/pointcloud_2.pcd");

  ROS_INFO("Registering Kinect to Model...");
  std::vector<cv::Mat> images;
  std::vector<Eigen::Matrix4f> transformations;
  io_obj_.loadImagesFromDir(ParameterServer::instance()->get<std::string> ("mesh_registration_images_directory"),images);
  io_obj_.loadTransformationsFromDir(ParameterServer::instance()->get<std::string> ("mesh_registration_transformations_directory"),transformations);
  KinectRegistration kinect_reg;
  Eigen::Matrix4f dildos;
  dildos = kinect_reg.registerKinectToModel(pointcloud_ptrs["model"],kinect_cloud_ptr,kinect_image,images,transformations);
  ROS_INFO_STREAM("final trafo \n " << dildos);
}
