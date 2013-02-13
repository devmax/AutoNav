#include "Controller.h"

int main(int argc, char** argv)
{
  ros::init(argc,argv,"drone_autopilot");

  ROS_INFO("Starting autopilot node now!");

  DroneController controller;

  controller.Loop();

  return 0;
}
