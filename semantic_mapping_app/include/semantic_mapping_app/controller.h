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

class Controller
{
  typedef std::pair<std::string, PointCloudConstPtr> NamedPointCloudPtr;

public:
    Controller();
    virtual ~Controller();

    void add_pointcloud(const std::string new_cloud_name, const PointCloudConstPtr new_cloud_ptr);

    void importScan();
    void alignToPrincipleAxis();
    void extractROI();
    void segmentPlanes();
    void registerKinectToModel();

private:
    ros::NodeHandle nh_;
    Visualization visualizer_;
    MeshIO io_obj_;
    std::map<std::string, PointCloudConstPtr> pointcloud_ptrs_;
    Eigen::Matrix4f align_to_axis_, move_model_to_origin_;

};

#endif
