/*
 * main.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: kidson
 */

#include <ros/ros.h>

#include "semantic_mapping_app/controller.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");

  Controller control_obj;
//  control_obj.importScan();
//  control_obj.alignToPrincipleAxis();
//  control_obj.extractROI();

  ros::Rate loop_rate (1000);
  while(ros::ok())
  {
    control_obj.spinVisualizer();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
