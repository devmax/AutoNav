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

ros::Publisher rpy_pub;
geometry_msgs::Point rpy;
void tagCB(const ar_track_alvar::AlvarMarkers &msg)
{
  for(size_t i=0;i<msg.markers.size();i++)
    {
      ar_track_alvar::AlvarMarker marker=msg.markers[i];
      double roll, pitch, yaw;
      tf::Pose tf_pose;

      tf::poseMsgToTF(marker.pose.pose,tf_pose);
      tf_pose.getBasis().getRPY(roll,pitch,yaw);
      rpy.x=roll;
      rpy.y=pitch;
      rpy.z=yaw;
      rpy_pub.publish(rpy);
    }
}
int main(int argc, char**argv)
{
  ros::init(argc,argv,"rpy");
  ros::NodeHandle n;
  ros::Subscriber tag_sub = n.subscribe("/ar_pose_marker",10,tagCB);
  rpy_pub = n.advertise<geometry_msgs::Point>("quatTOrpy",10);
  ros::spin();

  return 0;
}
