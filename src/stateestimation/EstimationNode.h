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
#include <vector>
#include <algorithm>
#include "AutoNav/filter_state.h"
#include "AutoNav/filter_var.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>

class DroneKalmanFilter;
class ScaleMap;

#ifndef _EIGEN_TYPES_
#define _EIGEN_TYPES_

typedef Eigen::Matrix<float,6,1> Vector6f;
typedef Eigen::Matrix<float,10,1> Vector10f;

const double PI = 3.14159265359;
#endif //_EIGEN_TYPES_

class EstimationNode
{

private:
  ros::Subscriber navdata_sub;
  ros::Subscriber control_sub;
  ros::Subscriber PTAM_sub;
  ros::Subscriber command_sub;
  ros::Time lastNavStamp;

  ros::Publisher dronepose_pub;
  ros::Publisher dronevar_pub;
  ros::Publisher currentstate_pub;
  ros::Publisher command_pub;

  ros::Publisher logTag_pub;
  ros::NodeHandle nh;

  ros::Duration predTime;
  int publishFreq;

  const static int nBuff = 30;
  Eigen::Matrix<double,nBuff,4> qBuff;
  int quatCounter;

  double Lx,Ly,Lz;

  double fuzzyThres;

  bool inited;
  tf::Transform origToWorld;

  std::string navdata_channel;
  std::string control_channel;
  std::string PTAM_channel;
  std::string output_channel;
  std::string current_output_channel;
  std::string command_channel;
  std::string variances_channel;

  tf::TransformBroadcaster state_broadcaster;

  ardrone_autonomy::Navdata lastNavdataReceived;

public:

  DroneKalmanFilter* filter;
  ScaleMap* scale;

  EstimationNode();

  void navdataCB(const ardrone_autonomy::NavdataConstPtr navdataPtr);
  void velCB(const geometry_msgs::TwistConstPtr controlPtr);
  void ptamCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr posePtr);
  void commandCB(const std_msgs::StringConstPtr comPtr);

  double getMedian(const Eigen::Matrix<double,nBuff,1> & data);
  
  void Loop();

};
#endif
