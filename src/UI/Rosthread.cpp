#include "Rosthread.h"

Rosthread::Rosthread()
{
  gui = NULL;
  velCount = 0;
  speed = turn = 1.0;
  keepRunning = true;
}

Rosthread::~Rosthread(void)
{

}

void Rosthread::startSystem()
{
  keepRunning = true;
  start();
}

void Rosthread::stopSystem()
{
  keepRunning = false;
  join();
}

void Rosthread::sendTakeOff()
{
  takeoff_pub.publish(std_msgs::Empty());
}

void Rosthread::sendLand()
{
  land_pub.publish(std_msgs::Empty());
}

void Rosthread::sendToggleState()
{
  toggleState_pub.publish(std_msgs::Empty());
}

void Rosthread::sendFlatTrim()
{
  ROS_INFO("Sending Flat Trim!");
  flattrim_srv.call(flattrim_srv_empty);
  ros::Duration(1).sleep();
}

void Rosthread::publishCommand(geometry_msgs::Twist cmd)
{
  //ROS_INFO("Publishing command!");
  vel_pub.publish(cmd);
}

void Rosthread::velCB(const geometry_msgs::TwistConstPtr cmd)
{
  //ROS_INFO("Velocity call back!");
  velCount++;
}

void Rosthread::autohover()
{
  std_msgs::String command;
  command.data = "hover";
  command_pub.publish(command);
}

void Rosthread::housefigure()
{
  std_msgs::String command;
  command.data = "house";
  command_pub.publish(command);
}

void Rosthread::revertmanual()
{
  std_msgs::String command;
  command.data = "manual";
  command_pub.publish(command);
}

void Rosthread::run()
{
  ROS_INFO("Starting ROS Thread!");

  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
  land_pub = n.advertise<std_msgs::Empty>("/ardrone/land",1);
  toggleState_pub = n.advertise<std_msgs::Empty>("/ardrone/reset",1);
  command_pub = n.advertise<std_msgs::String>("/AutoNav/commands",1);

  vel_sub = n.subscribe("/cmd_vel",5,&Rosthread::velCB,this);
  flattrim_srv = n.serviceClient<std_srvs::Empty>(n.resolveName("/ardrone/flattrim"),1);

  sendFlatTrim();

  speed = turn = 1.0;

  ros::Time last = ros::Time::now();

  while(keepRunning && n.ok())
    {
      while((ros::Time::now() - last) < ros::Duration(0.1))
	ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1 - (ros::Time::now() - last).toSec()));
      last = ros::Time::now();

      // if nothing on /cmd_vel, repeat!
      if(velCount == 0)
	publishCommand(gui->calcKBControl());
	  
      velCount = 0;

    }

  gui->close();

  if(n.ok())
    ros::shutdown();

  ROS_INFO("Exiting ROS Thread now!");

}
