#include "ThreadROS.h"

Rthread::Rthread()
{
  marker = NULL;
  velCount = 0;
  keepRunning = true;
}

Rthread::~Rthread(void)
{

}

void Rthread::startSystem()
{
  keepRunning = true;
  start();
}

void Rthread::stopSystem()
{
  keepRunning = false;
  join();
}

void Rthread::sendTakeOff()
{
  takeoff_pub.publish(std_msgs::Empty());
}

void Rthread::sendLand()
{
  land_pub.publish(std_msgs::Empty());
}

void Rthread::sendToggleState()
{
  toggleState_pub.publish(std_msgs::Empty());
}

void Rthread::sendFlatTrim()
{
  ROS_INFO("Sending Flat Trim!");
  flattrim_srv.call(flattrim_srv_empty);
  ros::Duration(1).sleep();
}

void Rthread::publishVelCommand(geometry_msgs::Twist cmd)
{
  cmd.linear.x = std::min(1.0,std::max(-1.0,cmd.linear.x));
  cmd.linear.y = std::min(1.0,std::max(-1.0,cmd.linear.y));
  cmd.linear.z = std::min(1.0,std::max(-1.0,cmd.linear.z));
  cmd.angular.z = std::min(1.0,std::max(-1.0,cmd.angular.z));
  //ROS_INFO("Publishing velocity command (%lf,%lf,%lf,%lf)!",cmd.linear.x,cmd.linear.y,cmd.linear.z,cmd.angular.z);
  vel_pub.publish(cmd);
}

void Rthread::velCB(const geometry_msgs::TwistConstPtr cmd)
{
  //ROS_INFO("Velocity call back!");
  velCount++;
}

void Rthread::autohover()
{
  std_msgs::String command;
  command.data = "hover";
  command_pub.publish(command);
}

void Rthread::housefigure()
{
  std_msgs::String command;
  command.data = "house";
  command_pub.publish(command);
}

void Rthread::revertmanual()
{
  std_msgs::String command;
  command.data = "manual";
  command_pub.publish(command);
}

void Rthread::publishCommand(Position move)
{
  if(std::abs(move.x) > 0.1 || std::abs(move.y) > 0.1 || std::abs(move.z) > 0.1 || std::abs(move.yaw) > 5)
    {
      ROS_INFO("Moving by (%lf,%lf,%lf,%lf)",move.x,move.y,move.z,move.yaw);

      std_msgs::String command;
      std::ostringstream s;
      s<<"move "<<move.x<<" "<<move.y<<" "<<move.z<<" "<<move.yaw;
      command.data = s.str();      
      command_pub.publish(command);
    }
}

void Rthread::run()
{
  ROS_INFO("Starting ROS Thread!");

  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
  land_pub = n.advertise<std_msgs::Empty>("/ardrone/land",1);
  toggleState_pub = n.advertise<std_msgs::Empty>("/ardrone/reset",1);
  command_pub = n.advertise<std_msgs::String>("/AutoNav/commands",1);

  vel_sub = n.subscribe("/cmd_vel",5,&Rthread::velCB,this);
  flattrim_srv = n.serviceClient<std_srvs::Empty>(n.resolveName("/ardrone/flattrim"),1);

  sendFlatTrim();

  ros::Time last = ros::Time::now();

  while(keepRunning && n.ok())
    {
      while((ros::Time::now() - last) < ros::Duration(0.1))
	ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1 - (ros::Time::now() - last).toSec()));
      last = ros::Time::now();
      
      // if nothing on /cmd_vel, repeat!
      if(velCount == 0)
	publishVelCommand(marker->calcControl());
	  
      velCount = 0;

    }

  if(n.ok())
    ros::shutdown();

  ROS_INFO("Exiting ROS Thread now!");

}
