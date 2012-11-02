/*
 * Segmentation.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: kidson
 */

#include "segmentation_interface/segmentation.h"


Segmentation::Segmentation ()
{
  // TODO Auto-generated constructor stub

}

Segmentation::~Segmentation ()
{
  // TODO Auto-generated destructor stub
}

void Segmentation::setInputCloud(const PointCloudConstPtr input)
{
  PointCloud temp = *input;
  input_cloud_ = temp.makeShared();
}
