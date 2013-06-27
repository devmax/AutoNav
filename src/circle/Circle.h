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

#define PI 3.14159265

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
  int angRes;
  double latVel;
  double angVel;

  PID atr;
  PID ctr;
  PID angular;

  ros::NodeHandle n;
  ros::Publisher vel;
  ros::Publisher log_control;
  ros::Subscriber state;

 public:
  Circle();
  void dynConfCB(AutoNav::CircleParamsConfig &config,uint32_t level);
  void stateCB(const AutoNav::filter_stateConstPtr state);
};
#endif //Circle.h
