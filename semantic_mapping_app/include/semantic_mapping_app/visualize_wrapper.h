/*
 * visualize_wrapper.h
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#ifndef VISUALIZE_WRAPPER_H_
#define VISUALIZE_WRAPPER_H_

//visualization
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

class VisualizeWrapper
{
  public:
    VisualizeWrapper ();
    virtual ~VisualizeWrapper ();
    void visualizeCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr);
  private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif /* VISUALIZE_WRAPPER_H_ */
