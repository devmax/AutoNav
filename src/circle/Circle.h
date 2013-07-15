#pragma once

#ifndef _CIRCLE_H_
#define _CIRCLE_H_

#include <ros/ros.h>
#include "AutoNav/filter_state.h"
#include "AutoNav/circle_control.h"
#include <dynamic_reconfigure/server.h>
#include "AutoNav/CircleParamsConfig.h"
#include "../HelperFunction.h"
#include <geometry_msgs/Twist.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <math.h>
#include "ar_track_alvar/AlvarMarker.h"
#include "ar_track_alvar/AlvarMarkers.h"

const double PI = 3.14159265;

struct PID
{
  double Kp,Kd,Ki;
  inline PID(){Kp=Kd=Ki=0;}
  inline PID(double Kp,double Kd,double Ki)
  {
    this->Kp = Kp;
    this->Kd = Kd;
    this->Ki = Ki;
  }
};

class Circle
{
 private:
  double radius;
  double latVel;
  double angVel;
  int dirn;

  PID atr;
  PID ctr;
  PID angular;

  bool radiusInit;
  bool stateInit;

  double initX,initY,initA;

  Eigen::Vector3f lastError;
  Eigen::Vector3f iTerm;
  ros::Time lastTime;

  ros::NodeHandle n;
  ros::Publisher vel;
  ros::Publisher log_control;
  ros::Subscriber state;

 public:
  Circle();
  void dynConfCB(AutoNav::CircleParamsConfig &config,uint32_t level);
  void stateCB(const AutoNav::filter_stateConstPtr state);
  void begin();
};
#endif //Circle.h
