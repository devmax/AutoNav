#include "EstimationNode.h"
#include "DroneKalmanFilter.h"
#include "ScaleMap.h"

using namespace std;

EstimationNode::EstimationNode()
{
  navdata_channel=nh.resolveName("/ardrone/navdata");
  control_channel=nh.resolveName("/cmd_vel");
  output_channel=nh.resolveName("/ardrone/predictedPose");
  current_output_channel = nh.resolveName("/ardrone/currentPose");
  PTAM_channel=nh.resolveName("/vslam/pose");
  command_channel=nh.resolveName("/AutoNav/commands");
  variances_channel=nh.resolveName("/ardrone/variances");

  predTime=ros::Duration(25*0.001);
  publishFreq=30;

  navdata_sub=nh.subscribe(navdata_channel,10,&EstimationNode::navdataCB,this);
  control_sub=nh.subscribe(control_channel,10,&EstimationNode::velCB,this);
  PTAM_sub=nh.subscribe(PTAM_channel,10,&EstimationNode::ptamCB,this);
  command_sub=nh.subscribe(command_channel,10,&EstimationNode::commandCB,this);
  
  dronepose_pub=nh.advertise<AutoNav::filter_state>(output_channel,1);
  dronevar_pub=nh.advertise<AutoNav::filter_var>(variances_channel,1);
  currentstate_pub=nh.advertise<AutoNav::filter_state>(current_output_channel,1);
  command_pub=nh.advertise<std_msgs::String>(command_channel,1);
  logTag_pub = nh.advertise<AutoNav::tags>("/log_tags",1);

  filter=new DroneKalmanFilter();
  lastNavStamp=ros::Time(0); 

  quatCounter = 1;
  fuzzyThres = 0.1;

  inited = false;

  Lx = Ly = Lz = -1;
}

void EstimationNode::commandCB(const std_msgs::StringConstPtr comPtr)
{
  if(comPtr->data.substr(0,7) == "Scale X")
    {
      scale = new ScaleMap(1.4,1);

      Lx = scale->getScale();

      filter->reset();

      filter->setScale(Lx,1);

      filter->setScale( (Ly == -1 ? Lx : Ly),2 );
      filter->setScale( (Lz == -1 ? Lx : Lz),3 );

      delete scale;

    }
  else if(comPtr->data.substr(0,7) == "Scale Y")
    {
      scale = new ScaleMap(1.4,2);

      Ly = scale->getScale();

      filter->reset();

      filter->setScale(Ly,2);

      filter->setScale( (Lx == -1 ? Ly : Lx),1 );
      filter->setScale( (Lz == -1 ? Ly : Lz),3 );

      delete scale;

    }
  else if(comPtr->data.substr(0,7) == "Scale Z")
    {
      scale = new ScaleMap(0.7,3);

      Lz = scale->getScale();

      filter->reset();

      filter->setScale(Lz,3);

      filter->setScale( (Lx == -1 ? Lz : Lx),1 );
      filter->setScale( (Ly == -1 ? Lz : Ly),2 );

      delete scale;

    }
  else if(comPtr->data.substr(0,5) == "Reset")
    filter->resetPoseVariances();
}

void EstimationNode::navdataCB(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
<<<<<<< HEAD
  //get Navdata message and push into observation queue of filter
  ROS_DEBUG("Navdata Call Back!");
=======

>>>>>>> 0339d1aef307420878e1cce4833a127c3740ab07
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

void EstimationNode::ptamCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr posePtr)
{
  tf::Transform droneToWorld = tf::Transform(tf::Quaternion(posePtr->pose.pose.orientation.x,posePtr->pose.pose.orientation.y,posePtr->pose.pose.orientation.z,posePtr->pose.pose.orientation.w),tf::Vector3(posePtr->pose.pose.position.x,posePtr->pose.pose.position.y,posePtr->pose.pose.position.z));

<<<<<<< HEAD
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
=======
  if(!inited)
    {
      origToWorld = droneToWorld;
      inited = true;
    }

  tf::Transform origToWorld_exp = (filter->getCurrentTF())*droneToWorld;
  tf::Quaternion q_ow = origToWorld_exp.getRotation();
  q_ow.normalize();
>>>>>>> 0339d1aef307420878e1cce4833a127c3740ab07

  tf::Transform origToDrone = origToWorld * (droneToWorld.inverse());

  Vector6f measurement;
  Vector6f variances;
  double roll,pitch,yaw;

  measurement(0) = origToDrone.getOrigin().x();
  measurement(1) = origToDrone.getOrigin().y();
  measurement(2) = origToDrone.getOrigin().z();

  tf::Matrix3x3(origToDrone.getRotation()).getRPY(roll,pitch,yaw);

<<<<<<< HEAD
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
=======
  measurement(3) = roll * 180 / PI;
  measurement(4) = pitch * 180 / PI;
  measurement(5) = yaw * 180 / PI;
>>>>>>> 0339d1aef307420878e1cce4833a127c3740ab07

  variances(0) = posePtr->pose.covariance[0];
  variances(1) = posePtr->pose.covariance[7];
  variances(2) = posePtr->pose.covariance[14];
  variances(3) = posePtr->pose.covariance[21];
  variances(4) = posePtr->pose.covariance[28];
  variances(5) = posePtr->pose.covariance[35];

  if(quatCounter > nBuff)
    {
<<<<<<< HEAD
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
=======

      if(quatCounter == nBuff+1)
	ROS_INFO("Quaternion buffer ready!");

      tf::Quaternion errQ = q_ow.inverse() * (tf::Quaternion(
							     getMedian(qBuff.block<nBuff,1>(0,0)),
							     getMedian(qBuff.block<nBuff,1>(0,1)),
							     getMedian(qBuff.block<nBuff,1>(0,2)),
							     getMedian(qBuff.block<nBuff,1>(0,3))
							     ).normalize());
      
      std::vector<double> coeffs;
      coeffs.push_back(errQ.x());
      coeffs.push_back(errQ.y());
      coeffs.push_back(errQ.z());
      coeffs.push_back(errQ.w());

      std::sort(coeffs.begin(),coeffs.end());

      if(std::max(coeffs.front(),-coeffs.back()) / std::abs(errQ.w()) * 2 > fuzzyThres)
	{
	  double r,p,y;
	  tf::Matrix3x3(errQ).getRPY(r,p,y);
	  ROS_WARN("Fuzzy tracking triggered for %lf,%lf,%lf",r*180/PI,p*180/PI,y*180/PI);

	  variances *= 3;

	  variances(3) *= 2;
	  variances(4) *= 2;
	  variances(5) *= 2;
	}
      else
	{
	  qBuff.block<1,4>(quatCounter-nBuff-1,0) = Eigen::Matrix<double,1,4>(q_ow.x(),q_ow.y(),q_ow.z(),q_ow.w());
	  quatCounter = quatCounter % nBuff + nBuff + 1;
>>>>>>> 0339d1aef307420878e1cce4833a127c3740ab07
	}
    }
  else
    {
      qBuff.block<1,4>(quatCounter-1,0) = Eigen::Matrix<double,1,4>(q_ow.x(),q_ow.y(),q_ow.z(),q_ow.w());
    }

<<<<<<< HEAD
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
=======
  ros::Time stamp;
  if(ros::Time::now()-posePtr->header.stamp > ros::Duration(30.0))
    stamp = ros::Time::now() - ros::Duration(0.001);
  else
    stamp = posePtr->header.stamp;
  
  pthread_mutex_lock(&filter->filter_CS);
  filter->addPTAM(measurement,variances,getMS(stamp)-filter->delayVideo);
  pthread_mutex_unlock(&filter->filter_CS);

}

double EstimationNode::getMedian(const Eigen::Matrix<double,nBuff,1> & data)
{
  std::vector<double> med;
  med.reserve(nBuff);
  for(int i=0; i<nBuff; i++)
    med.push_back(data(i));

  if(!med.empty())
    {
      std::vector<double>::iterator first = med.begin();
      std::vector<double>::iterator last = med.end();
      std::vector<double>::iterator mid = first + std::floor((last - first)/2);

      std::nth_element(first,mid,last);

      return *mid;
>>>>>>> 0339d1aef307420878e1cce4833a127c3740ab07
    }
  else 
    return 0;
}

void EstimationNode::Loop()
{

  ros::Rate pub_rate(publishFreq);

  while(nh.ok())
    {
      ros::spinOnce();

      pthread_mutex_lock(&filter->filter_CS);
<<<<<<< HEAD
      //      AutoNav::filter_state cur_s = filter->getPoseAt(ros::Time::now());
      AutoNav::filter_state s = filter->getPoseAt(ros::Time::now()+predTime);//always predict predTime ms in the future
=======

      AutoNav::filter_state s = filter->getPoseAt(ros::Time::now()+predTime);
      AutoNav::filter_var var = filter->getCurrentVariances();

>>>>>>> 0339d1aef307420878e1cce4833a127c3740ab07
      pthread_mutex_unlock(&filter->filter_CS);

      s.header.stamp=ros::Time::now();
      s.droneState=lastNavdataReceived.state;
      s.batteryPercent=lastNavdataReceived.batteryPercent;

      dronepose_pub.publish(s);
      dronevar_pub.publish(var);

      //state_broadcaster.sendTransform(tf::StampedTransform(filter->getCurrentTF(),ros::Time::now(),"/origin","/ardrone_base_link"));

      if((getMS(ros::Time::now())-filter->predictedUpToTimestamp)>500)
	filter->addFakePTAM(getMS(ros::Time::now())-300);
    
      pub_rate.sleep();
    }
}
