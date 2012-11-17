/*
 * MeshIO.h
 *
 *  Created on: Nov 17, 2012
 *      Author: kidson
 */

#ifndef MESHIO_H_
#define MESHIO_H_

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// pcl typedefs

#include "pcl_typedefs/pcl_typedefs.h"
#include "ros/ros.h"
#include <opencv2/core/core.hpp>

class MeshIO
{
  public:
    MeshIO ();
    virtual ~MeshIO ();

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

#endif /* MESHIO_H_ */
