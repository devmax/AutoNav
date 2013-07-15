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
  current_output_channel = nh.resolveName("/ardrone/currentPose");
  tag_channel=nh.resolveName("/ar_pose_marker");
  command_channel=nh.resolveName("/AutoNav/commands");

  predTime=ros::Duration(25*0.001);
  publishFreq=30;

  navdata_sub=nh.subscribe(navdata_channel,10,&EstimationNode::navdataCB,this);
  control_sub=nh.subscribe(control_channel,10,&EstimationNode::velCB,this);
  tag_sub=nh.subscribe(tag_channel,10,&EstimationNode::tagCB,this);
  
  dronepose_pub=nh.advertise<AutoNav::filter_state>(output_channel,1);
  currentstate_pub=nh.advertise<AutoNav::filter_state>(current_output_channel,1);
  command_pub=nh.advertise<std_msgs::String>(command_channel,1);

  filter=new DroneKalmanFilter();
  lastNavStamp=ros::Time(0); 

  lastID = 999;
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

void EstimationNode::tagCB(const ar_track_alvar::AlvarMarkersConstPtr tagsPtr)
{

  int lastTag=999, nextTag=999, minYawID=999;
  double minYaw = 999;
  tf::Transform droneToMarker;

  for(unsigned int i=0;i<tagsPtr->markers.size();i++)
    {
      if(tagsPtr->markers[i].id < 100)
	{
	  if(tagsPtr->markers[i].id == lastID)
	      lastTag = i;
	  else
	    {

	      const ar_track_alvar::AlvarMarker markerUsed = tagsPtr->markers[i];

	      tf::Quaternion q;
	      quaternionMsgToTF(markerUsed.pose.pose.orientation,q);
	      tf::Vector3 origin(markerUsed.pose.pose.position.x,markerUsed.pose.pose.position.y,markerUsed.pose.pose.position.z);
	      droneToMarker = tf::Transform(q,origin);

	      double r,p,y;

	      tf::Matrix3x3(droneToMarker.getRotation()).getRPY(r,p,y);

	      double yawDeg = (y*180/PI) - 90;

	      if(std::abs(yawDeg) < std::abs(minYaw))
		{
		  minYaw = yawDeg;
		  minYawID = i;
		}
	      if(yawDeg>10 && yawDeg<45)
		nextTag = i;	  
	    }
	}
    }

  ar_track_alvar::AlvarMarker markerUsed;
  bool transition = false;

  if(minYawID != 999 || lastTag!=999)
    {
      if(lastTag!=999)
	{
	  if(nextTag == 999)
	    {
	      markerUsed = tagsPtr->markers[lastTag];
	      ROS_INFO("Retained tag %d",markerUsed.id);
	    }
	  else
	    {
	      markerUsed = tagsPtr->markers[nextTag];
	      transition = true;
	      ROS_INFO("Ready to transition to tag %d",markerUsed.id);
	    }
	}
      else if(lastTag==999)
	{
	  markerUsed = tagsPtr->markers[minYawID];
	  transition = true;
	  ROS_INFO("Starting afresh from tag %d",markerUsed.id);
	}

      //      ROS_INFO("Using tag %d",markerUsed.id);
      lastID = markerUsed.id;

      tf::Quaternion q;
      quaternionMsgToTF(markerUsed.pose.pose.orientation,q);
      tf::Vector3 origin(markerUsed.pose.pose.position.x,markerUsed.pose.pose.position.y,markerUsed.pose.pose.position.z);
      droneToMarker = tf::Transform(q,origin);

      if(transition)
	{
	  ROS_INFO("Resetting origin for tags!");
	  
	  initToMarker = (filter->getCurrentTF()) * droneToMarker;
	}

      tf::Transform initToDrone = initToMarker*(droneToMarker.inverse());

      ros::Time stamp;
      if(ros::Time::now()-markerUsed.pose.header.stamp > ros::Duration(30.0))
	stamp=ros::Time::now()-ros::Duration(0.001);
      else
	stamp=markerUsed.pose.header.stamp;


      pthread_mutex_lock(&filter->filter_CS);
      filter->addTag(initToDrone,getMS(stamp)-filter->delayVideo);
      pthread_mutex_unlock(&filter->filter_CS);
    }

}

void EstimationNode::Loop()
{

  ros::Rate pub_rate(publishFreq);

  while(nh.ok())
    {
      ros::spinOnce();

      pthread_mutex_lock(&filter->filter_CS);
      //      AutoNav::filter_state cur_s = filter->getPoseAt(ros::Time::now());
      AutoNav::filter_state s = filter->getPoseAt(ros::Time::now()+predTime);
      pthread_mutex_unlock(&filter->filter_CS);

      s.header.stamp=ros::Time::now();
      s.droneState=lastNavdataReceived.state;
      s.batteryPercent=lastNavdataReceived.batteryPercent;

      dronepose_pub.publish(s);
      //      currentstate_pub.publish(cur_s);

      if((getMS(ros::Time::now())-filter->predictedUpToTimestamp)>500)
	filter->addFakeTag(getMS(ros::Time::now())-300);
    

      pub_rate.sleep();
    }
}
