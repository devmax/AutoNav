#pragma once

#ifndef _SCALE_MAP_H_
#define _SCALE_MAP_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "Eigen/Core"
#include "Eigen/LU"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "ardrone_autonomy/Navdata.h"
#include <tf/transform_listener.h>

#ifndef _EIGEN_TYPES_
#define _EIGEN_TYPES_

typedef Eigen::Matrix<float,6,1> Vector6f;
typedef Eigen::Matrix<float,10,1> Vector10f;

const double PI = 3.14159265359;
#endif //_EIGEN_TYPES_

class ScaleMap
{
 private:
  ros::NodeHandle n;

  ros::Subscriber navdata_sub;
  ros::Subscriber ptam_sub;

  ros::Publisher dbg;
  double lastNav;
  tf::Transform lastPtam;

  ros::Time lastNavStamp,lastPtamStamp;

  double dist;
  int axis;

 public:

  ScaleMap(double d,int a);

  void navdataCB(const ardrone_autonomy::NavdataConstPtr navdataPtr);
  void ptamCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr posePtr);

  double getScale();
};

#endif //ScaleMap.h
