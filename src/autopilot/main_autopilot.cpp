#include "ControlNode.h"
//#include "boost/thread.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"drone_autopilot");

  ROS_INFO("Starting autopilot node now!");

  ControlNode controlLoop;

  dynamic_reconfigure::Server<AutoNav::AutopilotParamsConfig> srv;
  dynamic_reconfigure::Server<AutoNav::AutopilotParamsConfig>::CallbackType f;
  f=boost::bind(&ControlNode::dynConfCB,&controlLoop,_1,_2);
  srv.setCallback(f);

  controlLoop.Loop();

  return 0;
}
