/*
 * controller.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: ross kidson
 */

#include "semantic_mapping_app/controller.h"
#include "semantic_mapping_app/parameter_server.h"

// includes from this stack
#include "kinect_capture_frame/kinectSnapshot.h"
#include "pcl_typedefs/pcl_typedefs.h"
#include "pcl_tools/pcl_tools.h"
#include "mesh_io/mesh_io.h"
#include "register_kinect_to_model/kinect_registration.h"
#include "align_principle_axis_interface/axis_alignment.h"
#include "segment_planes_interface/plane_segmentation.h"
#include "visualizer/visualization.h"

#include <ros/ros.h>
#include <pluginlib/class_loader.h>


//convert sensor msgs

#include <Eigen/Core>

Controller::Controller():
  nh_("~/controller"),
  visualizer_(),
  io_obj_(),
  reconfig_srv_(nh_)
{
  pluginlib::ClassLoader<segment_planes_interface::PlaneSegmentation> loader_planes("segment_planes_interface", "segment_planes_interface::PlaneSegmentation");
  try {
    plane_segmenter_ = loader_planes.createClassInstance("segment_planes_region_grow_plugin/PlaneSegmentationRegionGrow");
  }
  catch(pluginlib::PluginlibException& ex) {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  reconfig_callback_ = boost::bind (&Controller::reconfigCallback, this, _1, _2);
  reconfig_srv_.setCallback (reconfig_callback_);
}


Controller::~Controller()
{
    //destructor
}

void Controller::reconfigCallback (semantic_mapping_app::ControllerConfig &config,
    uint32_t level)
{
  //  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
  //            config.int_param, config.double_param,
  //            config.str_param.c_str(),
  //            config.bool_param?"True":"False",
  //            config.size);

  if(config.import_scan)
    this->importScan();
  if(config.align_to_principle_axis)
    this->alignToPrincipleAxis();
  if(config.extract_ROI)
    this->extractROI();
  if(config.extract_normals_from_model)
    this->extractNormalsFromModel();
  if(config.segment_planes)
    this->segmentPlanes();
  if(config.segment_fixtures)
    this->segmentFixtures();
  if(config.register_kinect_to_model)
    this->registerKinectToModel();

  config.import_scan = false;
  config.align_to_principle_axis = false;
  config.extract_ROI = false;
  config.extract_normals_from_model = false;
  config.segment_planes = false;
  config.segment_fixtures = false;
  config.register_kinect_to_model = false;

}

void Controller::spinVisualizer()
{
  visualizer_.spinOnce();
}

void Controller::add_pointcloud(const std::string new_cloud_name, const PointCloudConstPtr new_cloud_ptr)
{
  std::map<std::string,PointCloudConstPtr>::iterator existing_record = pointcloud_ptrs_.find(new_cloud_name);

  if(existing_record != pointcloud_ptrs_.end())   //pointcloud exists, update
    existing_record->second = new_cloud_ptr;
  else
    pointcloud_ptrs_.insert(NamedPointCloudPtr(new_cloud_name,new_cloud_ptr));
}

void Controller::add_pointcloud(const std::string new_cloud_name, const PointCloudNormalsConstPtr new_cloud_ptr)
{
  std::map<std::string,PointCloudNormalsConstPtr>::iterator existing_record = pointcloud_normals_ptrs_.find(new_cloud_name);

  if(existing_record != pointcloud_normals_ptrs_.end())   //pointcloud exists, update
    existing_record->second = new_cloud_ptr;
  else
    pointcloud_normals_ptrs_.insert(NamedPointCloudNormalsPtr(new_cloud_name,new_cloud_ptr));
}

void Controller::calculateNormals(const std::string cloud_name)
{
  ROS_INFO_STREAM("extracting normals for pointcloud " << cloud_name);
  std::map<std::string,PointCloudConstPtr>::iterator existing_record = pointcloud_ptrs_.find(cloud_name);

  if(existing_record != pointcloud_ptrs_.end())   //pointcloud exists, update
  {
    PointCloudNormalsPtr model_normals (new PointCloudNormals);
    pcl_tools::calculateNormalsOMP(existing_record->second, model_normals);
    add_pointcloud(cloud_name, model_normals);
  }
  else
    ROS_WARN_STREAM("pointcloud " << cloud_name << " does not exist in pointcloud_ptrs_ storage ");
}

void Controller::importScan()
{
  ROS_INFO("Importing mesh to pointcloud model...");
  PointCloudPtr raw_scan_pointcloud_ptr;
  raw_scan_pointcloud_ptr = io_obj_.loadMeshFromFile (ParameterServer::instance()->get<std::string>
                                            ("mesh_input_filename"));
  add_pointcloud("raw_scan",raw_scan_pointcloud_ptr);
  visualizer_.visualizeCloud(pointcloud_ptrs_["raw_scan"]);
}

void Controller::alignToPrincipleAxis()
{
  ROS_INFO("Performing principle axis alignment...");
  pluginlib::ClassLoader<align_principle_axis_interface::AxisAlignment> loader_axis("align_principle_axis_interface", "align_principle_axis_interface::AxisAlignment");
  align_principle_axis_interface::AxisAlignment* axis_align = NULL;
  try
  {
    axis_align = loader_axis.createClassInstance("align_principle_axis_floor_plugin/FloorAxisAlignment");
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
  axis_align->alignCloudPrincipleAxis(pointcloud_ptrs_["raw_scan"], guess, aligned_scan_ptr, align_to_axis_);
  add_pointcloud("aligned_scan",aligned_scan_ptr);
  visualizer_.visualizeCloud(pointcloud_ptrs_["aligned_scan"]);
}

void Controller::extractROI()
{
  ROS_INFO("Applying boxfilter to cloud..");
  PointCloudPtr cabinet_cloud_ptr (new PointCloud);
  Eigen::Vector4f min_point (0.9, 0.8, -3.0, 1);
  Eigen::Vector4f max_point (1.85, 1.4, -1.2, 1);
  pcl_tools::filterCloud (pointcloud_ptrs_["aligned_scan"], min_point, max_point, cabinet_cloud_ptr);

  ROS_INFO("move model to origin...");
  PointCloudPtr cabinet_centered_cloud_ptr (new PointCloud);
  pcl_tools::moveModelToOrigin(cabinet_cloud_ptr, cabinet_centered_cloud_ptr, move_model_to_origin_);
  add_pointcloud("model",cabinet_centered_cloud_ptr);
  visualizer_.visualizeCloud(pointcloud_ptrs_["model"]);
  //io_obj_.savePointcloudToFile(pointcloud_ptrs_["model"],"cabinet_model.pcd");
}

void Controller::extractNormalsFromModel()
{
  add_pointcloud("model_downsample", pcl_tools::downsampleCloud(pointcloud_ptrs_["model"],0.001));
  calculateNormals("model");
  calculateNormals("model_downsample");
  visualizer_.visualizeCloudNormals(pointcloud_ptrs_["model"], pointcloud_normals_ptrs_["model"]);
}

void Controller::segmentPlanes()
{
  ROS_INFO("segment planes...");
  if(pointcloud_normals_ptrs_.find("model") == pointcloud_normals_ptrs_.end())
    extractNormalsFromModel();
  std::vector<PointCloudConstPtr> plane_cloud_ptrs;
  std::vector<pcl17::ModelCoefficients::ConstPtr> plane_models;

  plane_segmenter_->setNormals(pointcloud_normals_ptrs_["model"]);
  plane_segmenter_->segmentPlanes(pointcloud_ptrs_["model"], plane_cloud_ptrs,plane_models);

  if(plane_cloud_ptrs.size() > 0)
    visualizer_.visualizeCloud(plane_cloud_ptrs);
}

void Controller::segmentFixtures()
{

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
  PointCloudPtr kinect_cloud_ptr = pcl_tools::convertSensorMsgPointCloudToPCL(get_kinect_frame_srv.response.pointcloud);
  cv::Mat kinect_image = pcl_tools::convertSensorMsgToCV(get_kinect_frame_srv.response.image);

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
  dildos = kinect_reg.registerKinectToModel(pointcloud_ptrs_["model"],kinect_cloud_ptr,kinect_image,images,transformations);
  ROS_INFO_STREAM("final trafo \n " << dildos);
}
