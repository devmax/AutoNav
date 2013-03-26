#include "Interact.h"
#include "ThreadROS.h"

Interact marker_interact;

int main(int argc,char **argv)
{
  ros::init(argc,argv,"interact");

  Rthread rthread;

  rthread.marker = &marker_interact;
  marker_interact.rosthread = &rthread;

  rthread.startSystem();

  marker_interact.start();

  return 0;
}
