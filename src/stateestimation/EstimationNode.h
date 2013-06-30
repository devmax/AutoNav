#pragma once

#ifndef _ESTIMATION_NODE_H
#define _ESTIMATION_NODE_H

#include <ros/ros.h>
#include "Eigen/Core"
#include "Eigen/LU"
#include "geometry_msgs/Twist.h"
#include "tf/tfMessage.h"
#include "../HelperFunction.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Empty.h"
#include <std_msgs/String.h>
#include "std_srvs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "ar_track_alvar/AlvarMarker.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include "AutoNav/filter_state.h"

class DroneKalmanFilter;

#ifndef _EIGEN_TYPES_
#define _EIGEN_TYPES_
typedef Eigen::Matrix<float,6,1> Vector6f;
typedef Eigen::Matrix<float,10,1> Vector10f;
#endif //_EIGEN_TYPES_

const double PI = 3.14159265359;

class EstimationNode
{

private:
  ros::Subscriber navdata_sub;
  ros::Subscriber control_sub;
  ros::Subscriber tag_sub;
  ros::Time lastNavStamp;

  ros::Publisher dronepose_pub;
  ros::Publisher currentstate_pub;
  ros::Publisher command_pub;

  ros::NodeHandle nh;

  ros::Duration predTime;
  int publishFreq;
  int numTags;
  int lastTag;
  double lastTag_y,nextTag_y;
  double tolerance;

  tf::Transform initToMarker;

  bool lastTag_found, nextTag_found;

  std::string navdata_channel;
  std::string control_channel;
  std::string tag_channel;
  std::string output_channel;
  std::string current_output_channel;
  std::string command_channel;

  ardrone_autonomy::Navdata lastNavdataReceived;

public:

  DroneKalmanFilter* filter;

  EstimationNode();

  void navdataCB(const ardrone_autonomy::NavdataConstPtr navdataPtr);
  void velCB(const geometry_msgs::TwistConstPtr controlPtr);
  void tagCB(const ar_track_alvar::AlvarMarkers &msg);
  
  void Loop();

};
#endif
