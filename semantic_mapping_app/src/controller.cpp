/*
 * controller.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: ross kidson
 */

#include "semantic_mapping_app/controller.h"

// includes from this stack
#include "kinect_capture_frame/kinectSnapshot.h"
#include "pcl_typedefs/pcl_typedefs.h"
#include "pcl_tools/pcl_tools.h"
#include "mesh_io/mesh_io.h"
#include "register_kinect_to_model/kinect_registration.h"
#include "rviz_visualizer/rviz_visualization.h"

// pluginlib interfaces
#include "align_principle_axis_interface/axis_alignment.h"
#include "segment_planes_interface/plane_segmentation.h"
#include "segment_fixtures_interface/fixture_segmentation.h"

#include <pcl/filters/extract_indices.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>


//convert sensor msgs

#include <Eigen/Core>

const static std::string segment_planes_plugin_name = "segment_planes_region_grow_plugin/PlaneSegmentationRegionGrow";
const static std::string segment_fixtures_plugin_name = "segment_fixtures_from_planes_plugin/FixtureSegmentationFromPlanes";
const static std::string align_axis_plugin_name = "align_principle_axis_floor_plugin/FloorAxisAlignment";

Controller::Controller():
  nh_("~/controller"),
  visualizer_(),
  io_obj_(),
  reconfig_srv_(nh_),
  plane_segmenter_ptr_(),
  fixture_segmenter_ptr_(),
  axis_align_ptr_(),
  loader_planes_("segment_planes_interface", "segment_planes_interface::PlaneSegmentation"),
  loader_fixtures_("segment_fixtures_interface", "segment_fixtures_interface::FixtureSegmentation")
{
  move_model_to_origin_ = Eigen::Matrix4f::Identity (4, 4);
  ROI_size_ = Eigen::Vector4f(0,0,0,1);
  loadSegmentPlanesPlugin(segment_planes_plugin_name);
  loadSegmentFixturesPlugin(segment_fixtures_plugin_name);
  loadAlignAxisPlugin(align_axis_plugin_name);
  reconfig_callback_ = boost::bind (&Controller::reconfigCallback, this, _1, _2);
  reconfig_srv_.setCallback (reconfig_callback_);
}


Controller::~Controller()
{
    //destructor
}

void Controller::loadSegmentPlanesPlugin(std::string plugin_name)
{
  if(plane_segmenter_ptr_ != NULL)
    plane_segmenter_ptr_.reset();
  ROS_INFO("create plane segment obj");
  try {
    plane_segmenter_ptr_ = loader_planes_.createInstance(plugin_name);
  }
  catch(pluginlib::PluginlibException& ex) {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
}

void Controller::loadSegmentFixturesPlugin(std::string plugin_name)
{
  if(fixture_segmenter_ptr_ != NULL)
    fixture_segmenter_ptr_.reset();
  ROS_INFO("create fix segment obj");
  try {
    fixture_segmenter_ptr_ = loader_fixtures_.createInstance(plugin_name);
  }
  catch(pluginlib::PluginlibException& ex) {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
}

void Controller::loadAlignAxisPlugin(std::string plugin_name)
{
  pluginlib::ClassLoader<align_principle_axis_interface::AxisAlignment> loader_axis("align_principle_axis_interface", "align_principle_axis_interface::AxisAlignment");
  try  {
    axis_align_ptr_ = loader_axis.createInstance(plugin_name);
  }
  catch(pluginlib::PluginlibException& ex)  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
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
  if(config.display_all_segmented_features)
    this->displayAllSegmentedFeatures();
  if(config.register_kinect_to_model)
    this->registerKinectToModel();
  if(config.reload_plane_segmenter)
    this->loadSegmentPlanesPlugin(segment_planes_plugin_name);
  if(config.reload_fixture_segmenter)
    this->loadSegmentFixturesPlugin(segment_fixtures_plugin_name);

  config.import_scan = false;
  config.align_to_principle_axis = false;
  config.extract_ROI = false;
  config.extract_normals_from_model = false;
  config.segment_planes = false;
  config.segment_fixtures = false;
  config.register_kinect_to_model = false;
  config.reload_plane_segmenter = false;
  config.reload_fixture_segmenter = false;
  config.display_all_segmented_features = false;

  move_model_to_origin_.block<3,1>(0,3) = -Eigen::Vector3f(config.ROI_origin_x, config.ROI_origin_y, config.ROI_origin_z);
  ROI_size_ = Eigen::Vector4f(config.ROI_size_x, config.ROI_size_y, config.ROI_size_z, 1);

  mesh_filename_ = config.mesh_filename;
  mesh_registration_images_directory_ = config.mesh_registration_images_directory;
  mesh_registration_transformations_directory_ = config.mesh_registration_transformations_directory;
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
  raw_scan_pointcloud_ptr = io_obj_.loadMeshFromFile (mesh_filename_);
  add_pointcloud("raw_scan",raw_scan_pointcloud_ptr);
  visualizer_.removeAllClouds();
  visualizer_.addCloudToVisualizer(pointcloud_ptrs_["raw_scan"]);
}

void Controller::alignToPrincipleAxis()
{
  ROS_INFO("Performing principle axis alignment...");

  PointCloudPtr aligned_scan_ptr (new PointCloud);
  axis_align_ptr_->alignCloudPrincipleAxis(pointcloud_ptrs_["raw_scan"], aligned_scan_ptr, align_to_axis_);
  add_pointcloud("aligned_scan",aligned_scan_ptr);
  visualizer_.removeAllClouds();
  visualizer_.addCloudToVisualizer(pointcloud_ptrs_["aligned_scan"]);
}

void Controller::extractROI()
{
  PointCloudPtr cabinet_cloud_ptr (new PointCloud);
  PointCloudPtr cabinet_centered_cloud_ptr (new PointCloud);

  ROS_INFO("move model to origin...");
  //ROS_INFO_STREAM("ROI origin " <<  move_model_to_origin_);
  //ROS_INFO_STREAM("ROI size " <<  ROI_size_);
  pcl_tools::transformPointCloud(pointcloud_ptrs_["aligned_scan"],cabinet_centered_cloud_ptr, move_model_to_origin_);
  ROS_INFO("Applying boxfilter to cloud..");
  //io_obj_.savePointcloudToFile(cabinet_centered_cloud_ptr, "moved_cloud"); 
  //pcl_tools::filterCloud (cabinet_centered_cloud_ptr, Eigen::Vector4f(-99,-99,-99,1), Eigen::Vector4f(99,99,99,1), cabinet_cloud_ptr);
  pcl_tools::filterCloud (cabinet_centered_cloud_ptr, Eigen::Vector4f(0,0,0,1), ROI_size_, cabinet_cloud_ptr);

  add_pointcloud("model",cabinet_cloud_ptr);
  visualizer_.removeAllClouds();
  visualizer_.addCloudToVisualizer(pointcloud_ptrs_["model"]);
}

void Controller::extractNormalsFromModel()
{
  add_pointcloud("model_downsample", pcl_tools::downsampleCloud(pointcloud_ptrs_["model"],0.001));
  calculateNormals("model");
  calculateNormals("model_downsample");
  visualizer_.removeAllClouds();
  visualizer_.addNormalsToVisualizer(pointcloud_ptrs_["model"], pointcloud_normals_ptrs_["model"]);
}

void Controller::segmentPlanes()
{
  ROS_INFO("segment planes...");
  if(pointcloud_normals_ptrs_.find("model") == pointcloud_normals_ptrs_.end())
    extractNormalsFromModel();


  plane_segmenter_ptr_->setNormals(pointcloud_normals_ptrs_["model"]);
  plane_segmenter_ptr_->segmentPlanes(pointcloud_ptrs_["model"], plane_indices_ptrs_,plane_models_);

  visualizer_.removeAllClouds();
  visualizer_.addCloudsToVisualizer(pointcloud_ptrs_["model"], plane_indices_ptrs_);
  //visualizer_.addCloudToVisualizer(pointcloud_ptrs_["model"], plane_indices_ptrs_[4]);

  for(int i=0; i<plane_models_.size(); i++)
  {
    ROS_INFO_STREAM("Plane models " << plane_models_[i]->values[0] << ", " <<
                        plane_models_[i]->values[1] << ", " <<
                        plane_models_[i]->values[2] << ", " <<
                        plane_models_[i]->values[3]);
  }
        // call complete plane function, passing also 2 points (to place the markers)
    visualizer_.completePlane(plane_models_[6], pointcloud_ptrs_["model"]->points[plane_indices_ptrs_[6]->indices.front()],
                                  pointcloud_ptrs_["model"]->points[plane_indices_ptrs_[6]->indices.back()]);
}


#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
void Controller::segmentFixtures()
{
  ROS_INFO("segment fixtures...");
  if(plane_models_.size() == 0)
  {
    ROS_INFO("No planes available for fixture extraction, please first run plane extraction");
    return;
  }

  fixture_segmenter_ptr_->setPlanes(plane_indices_ptrs_, plane_models_);
  fixture_segmenter_ptr_->segmentFixtures(pointcloud_ptrs_["model"], fixture_indices_ptrs_);

  visualizer_.removeAllClouds();
  visualizer_.addCloudsToVisualizer(pointcloud_ptrs_["model"], fixture_indices_ptrs_);

//  pcl::PointIndicesPtr inliers (new pcl::PointIndices());
//  pcl::ModelCoefficients coefficients;
//  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//  seg.setOptimizeCoefficients(true);
//  seg.setModelType(pcl::SACMODEL_PLANE);
//  seg.setMethodType(pcl::SAC_RANSAC);
//  seg.setDistanceThreshold(0.02);
//  seg.setInputCloud(pointcloud_ptrs_["model"]);
//  //        seg.setIndices(indices);
//  seg.segment(*inliers, coefficients);

//  std::vector<pcl::PointIndices> cluster_indices;
//  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
//  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(
//        new pcl::search::KdTree<pcl::PointXYZRGB>);

//  ec.setClusterTolerance(0.02);

//  ec.setMinClusterSize(100);
//  ec.setSearchMethod(kd_tree);
//  ec.setInputCloud(pointcloud_ptrs_["model"]);
//  ec.setIndices(inliers);
//  ec.extract(cluster_indices);

//  visualizer_.removeAllClouds();
//  visualizer_.addCloudToVisualizer(pointcloud_ptrs_["model"],inliers);
//  visualizer_.addCloudToVisualizer(pointcloud_ptrs_["model"],cluster_indices[0].makeShared());
}

void Controller::displayAllSegmentedFeatures()
{
  std::vector<pcl::PointIndicesConstPtr> feature_indices_ptrs;
  feature_indices_ptrs = plane_indices_ptrs_;
  for(std::vector<pcl::PointIndicesConstPtr>::const_iterator itr=fixture_indices_ptrs_.begin(); itr!=fixture_indices_ptrs_.end();itr++)
    feature_indices_ptrs.push_back(*itr);
  visualizer_.removeAllClouds();
  visualizer_.addCloudsToVisualizer(pointcloud_ptrs_["model"], feature_indices_ptrs);
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
  io_obj_.loadImagesFromDir(mesh_registration_images_directory_,images);
  io_obj_.loadTransformationsFromDir(mesh_registration_transformations_directory_,transformations);
  KinectRegistration kinect_reg;
  Eigen::Matrix4f dildos;
  dildos = kinect_reg.registerKinectToModel(pointcloud_ptrs_["model"],kinect_cloud_ptr,kinect_image,images,transformations);
  ROS_INFO_STREAM("final trafo \n " << dildos);
}
