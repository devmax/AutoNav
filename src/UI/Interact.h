#ifndef _INTERACT_H_
#define _INTERACT_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <math.h>
#include "ThreadROS.h"
#include <geometry_msgs/Pose.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <cmath>

#define PI 3.141592

class Rthread;

struct Position
{
  double x,y,z,yaw;
  inline Position(){x=y=z=yaw=0;}
  inline Position(double x,double y,double z,double yaw)
  {
    this->x = x;
    this->y = y;
    this->z = z;
    this->yaw = yaw;
  }
};

inline Position operator+(const Position a,const Position b)
{
  return Position(a.x+b.x,a.y+b.y,a.z+b.z,a.yaw+b.yaw);
}

inline Position operator-(const Position a,const Position b)
{
  return Position(a.x-b.x,a.y-b.y,a.z-b.z,a.yaw-b.yaw);
}

class Interact
{

 private:
  
 public:
  Interact();

  Rthread *rosthread;

  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler menu_handler_manual;
  visualization_msgs::InteractiveMarker int_marker;  
  visualization_msgs::InteractiveMarker manual_marker;  

  Position reference;
  Position manual_command;

  void start();

  Position poseToPosition(geometry_msgs::Pose pose);
  void setReference(geometry_msgs::Pose pose);
  Position findDifference(geometry_msgs::Pose pose);

  void resetManual();

  geometry_msgs::Twist calcControl();
  void generateCommandAuto(geometry_msgs::Pose pose);
  void generateCommandManual(geometry_msgs::Pose pose);

  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);

  void makeAutonomousMarker();
  void makeManualMarker();

};

extern Interact marker_interact;

#endif //Interact.h
