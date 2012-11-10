/*
 * mesh_converter.h
 *
 *  Created on: Nov 3, 2012
 *      Author: ross
 */

#ifndef MESH_CONVERTER_H_
#define MESH_CONVERTER_H_

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// pcl typedefs
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBNormal PointNormal;

typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

typedef pcl::PointCloud<PointNormal> PointCloudNormals;
typedef PointCloudNormals::Ptr PointCloudNormalsPtr;
typedef PointCloudNormals::ConstPtr PointCloudNormalsConstPtr;

#include "ros/ros.h"
#include "mesh_io/loadModelFromFile.h"
#include "mesh_io/loadImagesFromDir.h"
#include "mesh_io/loadTransformationsFromDir.h"
#include "mesh_io/loadPointcloudsFromDir.h"

#include "geometry_msgs/Transform.h"


class MeshConverter
{
  public:
    MeshConverter ();
    virtual ~MeshConverter ();

    void convertPCDtoMesh ();

    void convertMeshToPcd ();

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr loadMeshFromFile (std::string filename);

    PointCloudConstPtr loadPointcloudFromFile (std::string filename);

  private:
    ros::ServiceServer service_, service_images_, service_transforms_, service_pointclouds_;

    Eigen::Matrix4f extractTransformationFromFile (std::string filename);

    bool loadModelFromFileService (mesh_io::loadModelFromFile::Request &req,
        mesh_io::loadModelFromFile::Response &res);

    bool loadImagesFromDirService (mesh_io::loadImagesFromDir::Request &req,
        mesh_io::loadImagesFromDir::Response &res);

    bool loadPointcloudsFromDirService (mesh_io::loadPointcloudsFromDir::Request &req,
        mesh_io::loadPointcloudsFromDir::Response &res);

    bool loadTransformationsFromDirService (mesh_io::loadTransformationsFromDir::Request &req,
        mesh_io::loadTransformationsFromDir::Response &res);

    void getFileListWithExtension(const std::string& input_dir, const std::string& input_ext,
        std::set<std::string>& file_list);

    geometry_msgs::Transform convertMatrix4fToTF(const Eigen::Matrix4f& eigen_mat);
};

#endif /* MESH_CONVERTER_H_ */
