/*
 * MeshIO.h
 *
 *  Created on: Nov 17, 2012
 *      Author: kidson
 */

#ifndef MESHIO_H_
#define MESHIO_H_

// pcl typedefs

#include <set>
#include "pcl_typedefs/pcl_typedefs.h"
#include <opencv2/core/core.hpp>

class MeshIO
{
  public:
    MeshIO ();
    virtual ~MeshIO ();

    PointCloudPtr loadMeshFromFile (std::string filename);

    PointCloudPtr loadPointcloudFromFile (std::string filename);

    cv::Mat loadImageFromFile(std::string filename);

    void loadImagesFromDir (std::string directory, std::vector<cv::Mat>& images);

    void loadTransformationsFromDir (std::string directory, std::vector<Eigen::Matrix4f>& transforms);

    void loadPointcloudsFromDir (std::string directory, std::vector<PointCloud>& pointclouds);

  private:
    Eigen::Matrix4f extractTransformationFromFile (std::string filename);

    void getFileListWithExtension(const std::string& input_dir, const std::string& input_ext,
        std::set<std::string>& file_list);
};

#endif /* MESHIO_H_ */
