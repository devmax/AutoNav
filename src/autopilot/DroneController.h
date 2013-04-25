#pragma once

#ifndef _DRONE_CONTROLLER_H_
#define _DRONE_CONTROLLER_H_

#include <ros/ros.h>
#include "../HelperFunction.h"
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <string>
#include "AutoNav/control_commands.h"
#include "AutoNav/filter_state.h"
#include "../HelperFunction.h"

typedef Eigen::Matrix<float,4,1> Vector4f;

struct Position
{
  double x,y,z,yaw;
  inline Position(double x,double y,double z,double yaw)
  {
    this->x = x;
    this->y = y;
    this->z = z;
    this->yaw = yaw;
  }
  inline Position()
  {
    x=y=z=yaw = 0;
  }
};

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

class Controller
{
 private:
  Position goal;

  ros::NodeHandle nh;

  ros::Publisher cmd_pub;
  ros::Publisher log_control_commands;

  std::string velocity_channel;
  std::string log_control_commands_channel;
  
  double lastTimeStamp;
  double i_term;
  double last_error;
 public:

  PID rp;
  PID gaz;
  PID yaw;

  double initStayDist,stayWithinDist;
  int stayTimeMS,initReachClock;
  bool reached;

  double min_rp;
  double max_rp;

  double min_gaz;
  double max_gaz;

  double min_yaw;
  double max_yaw;

  double rise_fac;

  Controller();
  void setGoal(Position newGoal);
  void clearGoal();
  void sendControl(geometry_msgs::Twist cmd);
  geometry_msgs::Twist calcControl(Vector4f error,Vector4f d_error,double cur_yaw);
  bool update(const AutoNav::filter_stateConstPtr state);
  void i_increase(double new_err,double cap);
};

#endif //DroneController.h
