#ifndef _THREADROS_H_
#define _THREADROS_H_

#include "Interact.h"
#include "cvd/thread.h"
#include <unistd.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <stdio.h>

class Interact;
struct Position;

class Rthread : private CVD::Thread
{
 public:
  void run();

  ros::NodeHandle n;

  ros::Publisher vel_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher toggleState_pub;
  ros::Publisher command_pub;

  ros::Subscriber vel_sub;

  ros::ServiceClient flattrim_srv;

  std_srvs::Empty flattrim_srv_empty;

  unsigned int velCount;

  Rthread(void);
  ~Rthread(void);

  void startSystem();
  void stopSystem();

  Interact *marker;

  bool keepRunning;

  void publishVelCommand(geometry_msgs::Twist cmd);

  void velCB(const geometry_msgs::TwistConstPtr cmd);

  void autohover();
  void revertmanual();
  void publishCommand(Position move);
  void sendTakeOff();
  void housefigure();
  void sendLand();
  void sendToggleState();
  void sendFlatTrim();
};

#endif //ThreadROS.h
