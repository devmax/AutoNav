#include "EstimationNode.h"
#include "DroneKalmanFilter.h"
#include "deque"
#include <string>

using namespace std;

EstimationNode::EstimationNode()
{
  navdata_channel=nh.resolveName("/ardrone/navdata");
  control_channel=nh.resolveName("/cmd_vel");
  output_channel=nh.resolveName("/ardrone/predictedPose");
  tag_channel=nh.resolveName("/ar_pose_marker");

  predTime=ros::Duration(25*0.001);
  publishFreq=30;

  navdata_sub=nh.subscribe(navdata_channel,10,&EstimationNode::navdataCB,this);
  control_sub=nh.subscribe(control_channel,10,&EstimationNode::velCB,this);
  tag_sub=nh.subscribe(tag_channel,10,&EstimationNode::tagCB,this);
  
  dronepose_pub=nh.advertise<AutoNav::filter_state>(output_channel,1);

  filter=new DroneKalmanFilter();
  lastNavStamp=ros::Time(0); 
}

void EstimationNode::navdataCB(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
  ROS_DEBUG("Navdata Call Back!");
  lastNavdataReceived=*navdataPtr;
  if(ros::Time::now() - lastNavdataReceived.header.stamp > ros::Duration(30.0))
    lastNavdataReceived.header.stamp = ros::Time::now();

  pthread_mutex_lock(&filter->filter_CS);
  filter->navdataQueue->push_back(lastNavdataReceived);
  pthread_mutex_unlock(&filter->filter_CS);

  lastNavStamp=lastNavdataReceived.header.stamp;
}

void EstimationNode::velCB(const geometry_msgs::TwistConstPtr controlPtr)
{
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.twist=*controlPtr;

  pthread_mutex_lock(&filter->filter_CS);
  filter->velQueue->push_back(ts);
  pthread_mutex_unlock(&filter->filter_CS);
}

void EstimationNode::tagCB(const ar_track_alvar::AlvarMarkers &msg)
{
  ROS_DEBUG("Tag Call Back!");
  for(size_t i=0;i<msg.markers.size();i++)
    {
      ar_track_alvar::AlvarMarker marker=msg.markers[i];

      if(marker.id == 10)
	{
	  ros::Time stamp;
	  if(ros::Time::now()-marker.pose.header.stamp > ros::Duration(30.0))
	    stamp=ros::Time::now()-ros::Duration(0.001);
	  else
	    stamp=marker.pose.header.stamp;

	  tf::Quaternion q;
	  quaternionMsgToTF(marker.pose.pose.orientation,q);
	  tf::Vector3 origin(marker.pose.pose.position.x,marker.pose.pose.position.y,marker.pose.pose.position.z);
	  tf::Transform camToTag(q,origin);

	  pthread_mutex_lock(&filter->filter_CS);
	  filter->addTag(camToTag,getMS(stamp)-filter->delayVideo);
	  pthread_mutex_unlock(&filter->filter_CS);
	}
    }
}

void EstimationNode::Loop()
{
  ros::Rate pub_rate(publishFreq);

  while(nh.ok())
    {
      ros::spinOnce();

      pthread_mutex_lock(&filter->filter_CS);
      AutoNav::filter_state s = filter->getPoseAt(ros::Time::now()+predTime);
      pthread_mutex_unlock(&filter->filter_CS);

      s.header.stamp=ros::Time::now();
      s.droneState=lastNavdataReceived.state;
      s.batteryPercent=lastNavdataReceived.batteryPercent;

      dronepose_pub.publish(s);

      if((getMS(ros::Time::now())-filter->predictedUpToTimestamp)>500)
	filter->addFakeTag(getMS(ros::Time::now())-300);
    

      pub_rate.sleep();
    }
}
