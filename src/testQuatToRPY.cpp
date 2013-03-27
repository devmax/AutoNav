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

void setInitialFrame(geometry_msgs::Pose
void tagCB(const ar_track_alvar::AlvarMarkers &msg)
{
  for(size_t i=0;i<msg.markers.size();i++)
    {
      ar_track_alvar::AlvarMarker marker=msg.markers[i];

      tf::Quaternion quat;
      double roll,pitch,yaw;

      tf::quaternionMsgToTF(marker.pose.pose.orientation,quat);
      tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

      rpy.x=roll*180/3.14159268;
      rpy.y=pitch*180/3.14159268;
      rpy.z=yaw*180/3.14159268;
      rpy_pub.publish(rpy);
    }
}
int main(int argc, char**argv)
{
  ros::init(argc,argv,"rpy");
  ros::NodeHandle n;
  ros::Subscriber tag_sub = n.subscribe("/ar_pose_marker",10,tagCB);
  pose_pub = n.advertise<AutoNav::eulerpose>("/quatTOrpy",10);
  ros::spin();

  return 0;
}
