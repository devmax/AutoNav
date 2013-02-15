#ifndef _ROSTHREAD_H_
#define _ROSTHREAD_H_

#include "cvd/thread.h"
#include <unistd.h>
#include <ros/callback_queue.h>
#include "Teleop.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <stdio.h>

class Keypress;

class Rosthread : private CVD::Thread
{

 private:
  void run();

  ros::NodeHandle n;

  ros::Publisher vel_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher toggleState_pub;

  ros::Subscriber vel_sub;

  ros::ServiceClient flattrim_srv;

  std_srvs::Empty flattrim_srv_empty;

  unsigned int velCount;

 public:
  Rosthread(void);
  ~Rosthread(void);

  void startSystem();
  void stopSystem();

  Keypress *gui;

  double speed,turn;

  bool keepRunning;

  void publishCommand(geometry_msgs::Twist cmd);

  void velCB(const geometry_msgs::TwistConstPtr cmd);

  void sendTakeOff();
  void sendLand();
  void sendToggleState();
  void sendFlatTrim();
};

#endif //Rosthread.h
