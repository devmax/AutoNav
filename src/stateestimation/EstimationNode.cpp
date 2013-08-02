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
  logTag_pub = nh.advertise<AutoNav::tags>("/log_tags",1);

  filter=new DroneKalmanFilter();
  lastNavStamp=ros::Time(0); 

  lastID = 999;
}

void EstimationNode::navdataCB(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
  //get Navdata message and push into observation queue of filter
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
  // control commands sent are used to propogate state 
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
  tf::Transform droneToMarker; //drone=base frame, marker=target frame

  for(unsigned int i=0;i<tagsPtr->markers.size();i++)
    {
      if(tagsPtr->markers[i].id < 100)
	{
	  if(tagsPtr->markers[i].id == lastID)
	    lastTag = i; //store index of last tag seen
	  else
	    {
	      //if last tag is not found, we want to base our observation on the tag with the least yaw as seen from the drone's camera
	      const ar_track_alvar::AlvarMarker markerUsed = tagsPtr->markers[i];

	      tf::Quaternion q;
	      quaternionMsgToTF(markerUsed.pose.pose.orientation,q);
	      tf::Vector3 origin(markerUsed.pose.pose.position.x,markerUsed.pose.pose.position.y,markerUsed.pose.pose.position.z);
	      droneToMarker = tf::Transform(q,origin);

	      double r,p,y;

	      tf::Matrix3x3(droneToMarker.getRotation()).getRPY(r,p,y);

	      double yawDeg = (y*180/PI) - 90;

	      if(std::abs(yawDeg) < std::abs(minYaw))
		{//find the tag with the minimum yaw
		  minYaw = yawDeg;
		  minYawID = i;
		}
	      //these are tags that are ideal for transitioning to, i.e. assuming the drone is circling counter-clockwise, if a tag appears in the following configuration, it is a good idea to shift observation to this one now
	      if(yawDeg>10 && yawDeg<45)
		nextTag = i;	  
	    }
	}
    }

  ar_track_alvar::AlvarMarker markerUsed;
  bool transition = false;

  if(minYawID != 999 || lastTag!=999)
    {
      if(lastTag!=999) //last tag was found....
	{
	  if(nextTag == 999) //no appropriate tag found to transition to..
	    {
	      markerUsed = tagsPtr->markers[lastTag]; //simply observe using the last used tag
	      //ROS_INFO("Retained tag %d",markerUsed.id);
	    }
	  else
	    { //found a marker that is ideal for transitioning to
	      markerUsed = tagsPtr->markers[nextTag];
	      transition = true;
	      ROS_INFO("Ready to transition to tag %d",markerUsed.id);
	    }
	}
      else if(lastTag==999)
	{//didn't find last tag, just choose tag with lowest yaw w.r.t drone
	  markerUsed = tagsPtr->markers[minYawID];
	  transition = true;
	  ROS_INFO("Starting afresh from tag %d",markerUsed.id);
	}

      lastID = markerUsed.id; //store ID of tag used for future runs

      tf::Quaternion q;
      quaternionMsgToTF(markerUsed.pose.pose.orientation,q);
      tf::Vector3 origin(markerUsed.pose.pose.position.x,markerUsed.pose.pose.position.y,markerUsed.pose.pose.position.z);
      droneToMarker = tf::Transform(q,origin);

      AutoNav::tags log; //log all the details of the observation for debugging purposes

      log.markerID = markerUsed.id;

      tf::Transform current = filter->getCurrentTF(); //transform of worldToDrone (init=world, sorry)
      if(transition)
	{ //while transitioning to new tags or starting from a new tag, worldToMarker must be updated
	  initToMarker = current * droneToMarker;

	  log.initToMarker.linear.x = initToMarker.getOrigin().x();
	  log.initToMarker.linear.y = initToMarker.getOrigin().y();
	  log.initToMarker.linear.z = initToMarker.getOrigin().z();

	  double r,p,y;
	  tf::Matrix3x3(initToMarker.getRotation()).getRPY(r,p,y);

	  log.initToMarker.angular.x = r * 180 / PI;
	  log.initToMarker.angular.y = p * 180 / PI;
	  log.initToMarker.angular.z = y * 180 / PI;
	}

      log.droneToMarker.linear.x = droneToMarker.getOrigin().x();
      log.droneToMarker.linear.y = droneToMarker.getOrigin().y();
      log.droneToMarker.linear.z = droneToMarker.getOrigin().z();

      double r,p,y;
      tf::Matrix3x3(droneToMarker.getRotation()).getRPY(r,p,y);

      log.droneToMarker.angular.x = r * 180 / PI;
      log.droneToMarker.angular.y = p * 180 / PI;
      log.droneToMarker.angular.z = y * 180 / PI;

      tf::Transform initToDrone = initToMarker*(droneToMarker.inverse()); //worldToDrone = worldtoMarker * MarkertoDrone

      double curR,curP,curY;
      double measR,measP,measY;

      tf::Matrix3x3(current.getRotation()).getRPY(curR,curP,curY);
      tf::Matrix3x3(initToDrone.getRotation()).getRPY(measR,measP,measY);

      log.initToDrone.linear.x = initToDrone.getOrigin().x();
      log.initToDrone.linear.y = initToDrone.getOrigin().y();
      log.initToDrone.linear.z = initToDrone.getOrigin().z();
      log.initToDrone.angular.x = measR * 180 / PI;
      log.initToDrone.angular.y = measP * 180 / PI;
      log.initToDrone.angular.z = measY * 180 / PI;

      curY *= 180/PI;
      measY *= 180/PI;

      //a very hacky way of measuring sudden shifts in measurement, which means tracking has failed and the measurements are corrupted..must improve this
      if(std::abs(measY - curY) > 15)
	ROS_INFO("Sudden yaw from %lf to %lf",curY,measY);

      ros::Time stamp;
      if(ros::Time::now()-markerUsed.pose.header.stamp > ros::Duration(30.0))
	stamp=ros::Time::now()-ros::Duration(0.001); 
      else
	stamp=markerUsed.pose.header.stamp;

      pthread_mutex_lock(&filter->filter_CS);
      filter->addTag(
(Vector6f()<<log.initToDrone.linear.x,log.initToDrone.linear.y,log.initToDrone.linear.z,log.initToDrone.angular.x,log.initToDrone.angular.y,log.initToDrone.angular.z).finished(),
getMS(stamp)-filter->delayVideo);
      pthread_mutex_unlock(&filter->filter_CS);

      logTag_pub.publish(log);
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
      AutoNav::filter_state s = filter->getPoseAt(ros::Time::now()+predTime);//always predict predTime ms in the future
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
