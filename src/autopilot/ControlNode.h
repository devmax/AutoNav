#pragma once

#ifndef _CONTROL_NODE_H_
#define _CONTROL_NODE_H_

#include <ros/ros.h>
#include "AutoNav/filter_state.h"
#include <dynamic_reconfigure/server.h>
#include "AutoNav/AutopilotParamsConfig.h"
#include "DroneController.h"
#include <std_msgs/String.h>

inline Position operator+(const Position a,const Position b)
{
  return Position(a.x+b.x,a.y+b.y,a.z+b.z,a.yaw+b.yaw);
  //return sum;
}

inline Position operator-(const Position a,const Position b)
{
  return Position(a.x-b.x,a.y-b.y,a.z-b.z,a.yaw-b.yaw);
  //return diff;
}

class ControlNode
{
 private:
  ros::NodeHandle n;

  ros::Subscriber state_sub;
  ros::Subscriber command_sub;

  std::string state_channel;
  std::string command_channel;

  std::string current;

  bool goalSet;
  bool goalReached;  

 Controller controller;
 public:
  ControlNode();
  void Loop();
  void commandCB(const std_msgs::StringConstPtr str);
  void stateCB(const AutoNav::filter_stateConstPtr state);
  void dynConfCB(AutoNav::AutopilotParamsConfig &config, uint32_t level);
  Position stateToPosition(const AutoNav::filter_stateConstPtr state);
  void beginHover(Position goal);
};

#endif //ControlNode.h
