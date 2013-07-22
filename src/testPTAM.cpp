#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include "AutoNav/eulerpose.h"

const double PI = 3.14159268;

int main(int argc,char** argv)
{
  ros::init(argc,argv,"check_ptam");

  ros::NodeHandle nh;
  ros::Publisher pose_pub;

  pose_pub = nh.advertise<AutoNav::eulerpose>("/PTAMpose",10);

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  tf::Transform initToWorld;
  bool init = false;

  while(nh.ok())
    {
      tf::StampedTransform droneToWorld;

      try
	{
	  listener.waitForTransform("/ardrone_base_link","/world",ros::Time(0),ros::Duration(1.0));
	  listener.lookupTransform("/ardrone_base_link","/world",ros::Time(0),droneToWorld);
	}
      catch (tf::TransformException ex)
	{
	  ROS_ERROR("%s",ex.what());
	}

      if(!init)
	{
	  initToWorld = droneToWorld;
	  init = true;
	}

      tf::Transform initToDrone = initToWorld*(droneToWorld.inverse());

      broadcaster.sendTransform(tf::StampedTransform(initToDrone,ros::Time::now(),"/origin","/ardrone_base_link"));

      double roll,pitch,yaw;

      tf::Matrix3x3(initToDrone.getRotation()).getRPY(roll,pitch,yaw);

      AutoNav::eulerpose epose;

      epose.position.x = initToDrone.getOrigin().x();
      epose.position.y = initToDrone.getOrigin().y();
      epose.position.z = initToDrone.getOrigin().z();
      epose.euler.x = roll * 180/ PI;
      epose.euler.y = pitch * 180 / PI;
      epose.euler.z = yaw * 180 / PI;

      pose_pub.publish(epose);
    }

  return 0;
}
