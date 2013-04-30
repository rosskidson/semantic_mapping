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

#include "pcl_typedefs/pcl_typedefs.h"
#include "ros/ros.h"

class MeshConverter
{
  public:
    MeshConverter ();
    virtual ~MeshConverter ();

    void convertPCDtoMesh ();

    void convertMeshToPcd ();

};

#endif /* MESH_CONVERTER_H_ */
