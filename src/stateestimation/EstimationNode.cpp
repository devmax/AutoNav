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

  lastTag = -255;
  lastTag_found = false;
  nextTag_found = false;

  lastTag_y = 0;
  nextTag_y = 0;

  tolerance = 10;

  numTags = 6;
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
  //  ROS_INFO("Tag Call Back!");
  ar_track_alvar::AlvarMarker markerUsed;
  tf::Transform droneToMarker, droneToNMarker, initToDrone;

  for(size_t i=0;i<msg.markers.size(); i++)
    {
      if(lastTag<0 && msg.markers[i].id<20)
	{
	  markerUsed = msg.markers[i];

	  if(markerUsed.id!=0)
	    continue;

	  tf::Quaternion q;
	  quaternionMsgToTF(markerUsed.pose.pose.orientation,q);
	  tf::Vector3 origin(markerUsed.pose.pose.position.x,markerUsed.pose.pose.position.y,markerUsed.pose.pose.position.z);
	  initToMarker = tf::Transform(q,origin);
	  droneToMarker = initToMarker;

	  lastTag = msg.markers[i].id;
	  lastTag_found = true;

	  ROS_INFO("First observation of tag %d made and offsets initialized...!",msg.markers[i].id);
	  break;
	}

      if(msg.markers[i].id == lastTag)
	{
	  lastTag_found = true;
	  markerUsed = msg.markers[i];

	  tf::Quaternion q;
	  quaternionMsgToTF(markerUsed.pose.pose.orientation,q);
	  tf::Vector3 origin(markerUsed.pose.pose.position.x,markerUsed.pose.pose.position.y,markerUsed.pose.pose.position.z);
	  droneToMarker = tf::Transform(q,origin);

	  lastTag = msg.markers[i].id;
	  //ROS_INFO("Last tag is %d",lastTag);

	  double r,p,y;
	  tf::Matrix3x3(droneToMarker.getRotation()).getRPY(r,p,y);

	  lastTag_y = y * 180/PI;
	}

      if(msg.markers[i].id == (lastTag+1)%numTags)
	{
	  nextTag_found = true;
	  markerUsed = msg.markers[i];

	  tf::Quaternion q;
	  quaternionMsgToTF(markerUsed.pose.pose.orientation,q);
	  tf::Vector3 origin(markerUsed.pose.pose.position.x,markerUsed.pose.pose.position.y,markerUsed.pose.pose.position.z);
	  droneToNMarker = tf::Transform(q,origin);

	  double r,p,y;
	  tf::Matrix3x3(droneToNMarker.getRotation()).getRPY(r,p,y);

	  nextTag_y = y * 180/PI;
	}
    }


  if(lastTag_found)
    {

      if(nextTag_found && std::abs(std::abs(lastTag_y) - std::abs(nextTag_y)) < tolerance)
	{
	  initToMarker *= (droneToMarker.inverse())*droneToNMarker;
	  droneToMarker = droneToNMarker;

	  ROS_INFO("Transitioning from tag %d to %d now!",lastTag,(lastTag+1)%numTags);

	  lastTag = (lastTag+1)%numTags;
	}

      initToDrone = initToMarker*(droneToMarker.inverse());

      ros::Time stamp;
      if(ros::Time::now()-markerUsed.pose.header.stamp > ros::Duration(30.0))
	stamp=ros::Time::now()-ros::Duration(0.001);
      else
	stamp=markerUsed.pose.header.stamp;


      pthread_mutex_lock(&filter->filter_CS);
      filter->addTag(initToDrone,getMS(stamp)-filter->delayVideo);
      pthread_mutex_unlock(&filter->filter_CS);

    }
  else
    {
      if(nextTag_found)
	ROS_INFO("New tag found but last tag not found...!");
      ROS_INFO("ERROR! Last tag %d lost without transition...!",lastTag);
    }


  lastTag_found = nextTag_found = false;
}

void EstimationNode::Loop()
{

  ros::Rate pub_rate(publishFreq);

  while(nh.ok())
    {
      ros::spinOnce();

      pthread_mutex_lock(&filter->filter_CS);
      AutoNav::filter_state cur_s = filter->getPoseAt(ros::Time::now());
      AutoNav::filter_state s = filter->getPoseAt(ros::Time::now()+predTime);
      pthread_mutex_unlock(&filter->filter_CS);

      s.header.stamp=ros::Time::now();
      s.droneState=lastNavdataReceived.state;
      s.batteryPercent=lastNavdataReceived.batteryPercent;

      dronepose_pub.publish(s);
      currentstate_pub.publish(cur_s);

      if((getMS(ros::Time::now())-filter->predictedUpToTimestamp)>500)
	filter->addFakeTag(getMS(ros::Time::now())-300);
    

      pub_rate.sleep();
    }
}
