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
typedef pcl::PointXYZRGB       PointType;
typedef pcl::PointXYZRGBNormal PointNormal;

typedef pcl::PointCloud<PointType>  PointCloud;
typedef PointCloud::Ptr             PointCloudPtr;
typedef PointCloud::ConstPtr        PointCloudConstPtr;

typedef pcl::PointCloud<PointNormal>  PointCloudNormals;
typedef PointCloudNormals::Ptr        PointCloudNormalsPtr;
typedef PointCloudNormals::ConstPtr   PointCloudNormalsConstPtr;

#include "ros/ros.h"
#include "mesh_io/loadModelFromFile.h"

class MeshConverter {
public:
	MeshConverter();
	virtual ~MeshConverter();

	void convertPCDtoMesh ();

	void convertMeshToPcd();

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr loadMeshFromFile(std::string filename);

	PointCloudConstPtr loadPointcloudFromFile(std::string filename);

private:
	ros::ServiceServer service_;

	bool loadModelFromFileService(mesh_io::loadModelFromFile::Request  &req,
	    mesh_io::loadModelFromFile::Response &res );
};

#endif /* MESH_CONVERTER_H_ */
