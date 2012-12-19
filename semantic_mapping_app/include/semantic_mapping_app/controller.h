/*
 * controller.h
 *
 *  Created on: Nov 26, 2012
 *      Author: ross kidson
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>

#include "mesh_io/mesh_io.h"
#include "visualizer/visualization.h"
#include "pcl_typedefs/pcl_typedefs.h"

#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/semantic_mapping_app/ControllerConfig.h"

#include "segment_planes_interface/plane_segmentation.h"
#include "segment_fixtures_interface/fixture_segmentation.h"

#include <pluginlib/class_loader.h>

class Controller
{
  typedef std::pair<std::string, PointCloudConstPtr> NamedPointCloudPtr;
  typedef std::pair<std::string, PointCloudNormalsConstPtr> NamedPointCloudNormalsPtr;

public:
    Controller();
    virtual ~Controller();
    void spinVisualizer();

    void importScan();
    void alignToPrincipleAxis();
    void extractROI();
    void extractNormalsFromModel();
    void segmentPlanes();
    void registerKinectToModel();
    void segmentFixtures();

private:

    void loadSegmentPlanesPlugin(std::string plugin_name);
    void loadSegmentFixturesPlugin(std::string plugin_name);

    void add_pointcloud(const std::string new_cloud_name, const PointCloudConstPtr new_cloud_ptr);
    void add_pointcloud(const std::string new_cloud_name, const PointCloudNormalsConstPtr new_cloud_ptr);
    void calculateNormals(const std::string cloud_name);

    void reconfigCallback (semantic_mapping_app::ControllerConfig &config,
        uint32_t level);

    ros::NodeHandle nh_;
    Visualization visualizer_;
    MeshIO io_obj_;

    // data storage
    std::map<std::string, PointCloudConstPtr> pointcloud_ptrs_;
    std::map<std::string, PointCloudNormalsConstPtr> pointcloud_normals_ptrs_;
    std::vector<pcl17::PointIndicesConstPtr> plane_indices_ptrs_, fixture_indices_ptrs_;
    std::vector<pcl17::ModelCoefficients::ConstPtr> plane_models_;

    Eigen::Matrix4f align_to_axis_, move_model_to_origin_;
    Eigen::Vector4f ROI_size_;
    std::string mesh_filename_, mesh_registration_images_directory_, mesh_registration_transformations_directory_;

    dynamic_reconfigure::Server<semantic_mapping_app::ControllerConfig> reconfig_srv_;
    dynamic_reconfigure::Server<semantic_mapping_app::ControllerConfig>::CallbackType
          reconfig_callback_;

    //plugin pointers
    boost::shared_ptr<segment_planes_interface::PlaneSegmentation> plane_segmenter_ptr_;
    boost::shared_ptr<segment_fixtures_interface::FixtureSegmentation> fixture_segmenter_ptr_;

    pluginlib::ClassLoader<segment_planes_interface::PlaneSegmentation> loader_planes_;
    pluginlib::ClassLoader<segment_fixtures_interface::FixtureSegmentation> loader_fixtures_;

};

#endif
