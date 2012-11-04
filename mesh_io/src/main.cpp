/*
 * main.cpp
 *
 * mesh io node
 *
 *  Created on: Oct 30, 2012
 *      Author: rosskidson
 */

#include <ros/ros.h>
#include "mesh_io/mesh_converter.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "mesh_io");

  MeshConverter mesh_obj;
  ros::spin();

//  ros::Rate loop_rate (100);
//  while (ros::ok ())
//  {
//    ros::spinOnce ();
//    loop_rate.sleep ();
//  }

   return 0;

}
