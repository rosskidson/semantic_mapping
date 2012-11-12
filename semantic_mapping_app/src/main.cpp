/*
 * main.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include <ros/ros.h>


int main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");
  ros::NodeHandle nh;
  Visualization visualizer;

  geometry_msgs::Vector3 min_point, max_point;
  min_point.x = 1.0;
  min_point.y = 1.0;
  min_point.z = 0.6;
  max_point.x = 2.0;
  max_point.y = 2.7;
  max_point.z = 1.2;


  //visualizer.visualizeCloud(load_model.response.pointcloud);

  return 0;
}
