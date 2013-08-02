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

  if(!inited)
    {
      origToWorld = droneToWorld;
      inited = true;
    }

  tf::Transform origToWorld_exp = (filter->getCurrentTF())*droneToWorld;
  tf::Quaternion q_ow = origToWorld_exp.getRotation();
  q_ow.normalize();

  tf::Transform origToDrone = origToWorld * (droneToWorld.inverse());

  Vector6f measurement;
  Vector6f variances;
  double roll,pitch,yaw;

  measurement(0) = origToDrone.getOrigin().x();
  measurement(1) = origToDrone.getOrigin().y();
  measurement(2) = origToDrone.getOrigin().z();

  tf::Matrix3x3(origToDrone.getRotation()).getRPY(roll,pitch,yaw);

  measurement(3) = roll * 180 / PI;
  measurement(4) = pitch * 180 / PI;
  measurement(5) = yaw * 180 / PI;

  variances(0) = posePtr->pose.covariance[0];
  variances(1) = posePtr->pose.covariance[7];
  variances(2) = posePtr->pose.covariance[14];
  variances(3) = posePtr->pose.covariance[21];
  variances(4) = posePtr->pose.covariance[28];
  variances(5) = posePtr->pose.covariance[35];

  if(quatCounter > nBuff)
    {
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

	}
    }
  else
    {
      qBuff.block<1,4>(quatCounter-1,0) = Eigen::Matrix<double,1,4>(q_ow.x(),q_ow.y(),q_ow.z(),q_ow.w());
    }

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

      AutoNav::filter_state s = filter->getPoseAt(ros::Time::now()+predTime);
      AutoNav::filter_var var = filter->getCurrentVariances();

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
