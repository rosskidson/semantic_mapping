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
  control_obj.importScan();
  control_obj.alignToPrincipleAxis();
  control_obj.extractROI();
  //ros::spin();

  return 0;
}
