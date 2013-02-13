#include<ros/ros.h>
#include "EstimationNode.h"

unsigned int ros_header_timestamp_base=0;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"drone_stateestimation");

  ROS_INFO("Now starting state estimation node!");

  EstimationNode estimator;

  estimator.Loop();

  return 0;
}
