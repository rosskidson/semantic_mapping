/*
 * kinect_registration.h
 *
 *  Created on: Nov 4, 2012
 *      Author: kidson
 */

#ifndef KINECT_REGISTRATION_H_
#define KINECT_REGISTRATION_H_

#include "register_kinect_to_model/registerKinectToModel.h"
#include "ros/ros.h"

class KinectRegistration
{
  public:
    KinectRegistration ();
    virtual ~KinectRegistration ();

  private:
    bool registerKinectToModel (register_kinect_to_model::registerKinectToModel::Request &req,
        register_kinect_to_model::registerKinectToModel::Response &res);

    void registerKinectToModel();

    ros::ServiceServer service_;
};

#endif /* KINECT_REGISTRATION_H_ */
