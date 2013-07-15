#include<ros/ros.h>
#include<Eigen/Core>
#include<Eigen/LU>
#include<tf/tfMessage.h>
#include<tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include "ar_track_alvar/AlvarMarker.h"
#include "ardrone_autonomy/Navdata.h"
#include<geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include "AutoNav/eulerpose.h"

ros::Publisher pose_pub;
AutoNav::eulerpose pose_euler;
tf::Transform init_to_marker;
bool inited;

void init(geometry_msgs::Pose pose)
{
  ROS_INFO("Establishing initial transform!");
  tf::Quaternion q;
  quaternionMsgToTF(pose.orientation,q);
  tf::Vector3 origin(pose.position.x,pose.position.y,pose.position.z);

  init_to_marker = tf::Transform(q,origin);

  inited = true;
}
void tagCB(const ar_track_alvar::AlvarMarkers &msg)
{
  for(size_t i=0;i<msg.markers.size();i++)
    {
      ar_track_alvar::AlvarMarker marker=msg.markers[i];
      
      if(marker.id < 100)
	{
	  tf::Quaternion q;
	  quaternionMsgToTF(marker.pose.pose.orientation,q);
	  tf::Vector3 origin(marker.pose.pose.position.x,marker.pose.pose.position.y,marker.pose.pose.position.z);

	  /* UNCOMMENT FROM HERE FOR INIT_TO_DRONE
	  if(!inited)
	    {
	      init(marker.pose.pose);
	    }

	  tf::Transform drone_to_marker(q,origin);

	  tf::Transform init_to_drone = init_to_marker*(drone_to_marker.inverse());*/

	  tf::Transform init_to_drone(q,origin); //COMMENT THIS

	  double roll,pitch,yaw;

	  tf::Matrix3x3(init_to_drone.getRotation()).getRPY(roll,pitch,yaw);
      
	  AutoNav::eulerpose epose;

	  epose.position.x = init_to_drone.getOrigin().x();
	  epose.position.y = init_to_drone.getOrigin().y();
	  epose.position.z = init_to_drone.getOrigin().z();
	  epose.euler.x=roll*180/3.14159268;
	  epose.euler.y=pitch*180/3.14159268;
	  epose.euler.z=yaw*180/3.14159268 - 90;

	  pose_pub.publish(epose);
	}
    }
}

int main(int argc, char**argv)
{
  ros::init(argc,argv,"testRPY");

  inited = false;

  ros::NodeHandle n;
  ros::Subscriber tag_sub = n.subscribe("/ar_pose_marker",10,tagCB);
  pose_pub = n.advertise<AutoNav::eulerpose>("/quatTOrpy",10);
  ros::spin();

  return 0;
}
