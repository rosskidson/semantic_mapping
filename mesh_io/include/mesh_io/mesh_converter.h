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
typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGBNormal PointNormal;

typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

typedef pcl::PointCloud<PointNormal> PointCloudNormals;
typedef PointCloudNormals::Ptr PointCloudNormalsPtr;
typedef PointCloudNormals::ConstPtr PointCloudNormalsConstPtr;

#include "ros/ros.h"
#include <opencv2/core/core.hpp>

class MeshConverter
{
  public:
    MeshConverter ();
    virtual ~MeshConverter ();

    void convertPCDtoMesh ();

    void convertMeshToPcd ();

    PointCloudConstPtr loadMeshFromFile (std::string filename);

    PointCloudConstPtr loadPointcloudFromFile (std::string filename);

  private:
    Eigen::Matrix4f extractTransformationFromFile (std::string filename);

    void loadImagesFromDir (std::string directory, std::vector<cv::Mat>& images);

    void loadTransformationsFromDir (std::string directory, std::vector<Eigen::Matrix4f>& transforms);

    void loadPointcloudsFromDir (std::string directory, std::vector<pcl::PointCloud<pcl::PointXYZ> >& pointclouds);

    void getFileListWithExtension(const std::string& input_dir, const std::string& input_ext,
        std::set<std::string>& file_list);
};

#endif /* MESH_CONVERTER_H_ */
