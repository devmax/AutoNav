#include<ros/ros.h>
#include<Eigen/Core>
#include<Eigen/LU>
#include<tf/tfMessage.h>
#include<tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include "ar_track_alvar/AlvarMarker.h"
#include "ardrone_autonomy/Navdata.h"
#include "AutoNav/filter_state.h"
#include<geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

const double PI = 3.14159265359;

void poseCB(const AutoNav::filter_stateConstPtr state)
{
  tf::Transform droneState;
  tf::TransformBroadcaster tf_broadcaster;

  droneState.setRotation(tf::createQuaternionFromRPY(state->roll*PI/180,state->pitch*PI/180,state->yaw*PI/180));
  droneState.setOrigin(tf::Vector3(state->x,state->y,state->z));

  tf_broadcaster.sendTransform(tf::StampedTransform(droneState,ros::Time::now(),"/ardrone_base_link","drone"));
}
int main(int argc,char **argv)
{
  ros::init(argc,argv,"testTF");

  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("/ardrone/predictedPose",1,poseCB);

  ros::spin();

  return 0;
}
