#include "ScaleMap.h"

ScaleMap::ScaleMap(double d,int a)
{
  navdata_sub = n.subscribe("/ardrone/navdata",1,&ScaleMap::navdataCB,this);
  ptam_sub = n.subscribe("/vslam/pose",1,&ScaleMap::ptamCB,this);

  dist = d;
  axis = a;

  lastNavStamp = ros::Time::now() - ros::Duration(5);
  lastPtamStamp = ros::Time::now() - ros::Duration(10);

  listener = new tf::TransformListener(n);
}

void ScaleMap::navdataCB(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
  switch(axis)
    {
    case 1:
      lastNav = (double)(navdataPtr->vx)/1000;
      break;
    case 2:
      lastNav = (double)(navdataPtr->vy)/1000;
      break;
    case 3:
      lastNav = (double)(navdataPtr->altd)/1000;
      break;
    case 4:
      ROS_INFO("Invalid axis value entered for scale calibration");
    }

  lastNavStamp = navdataPtr->header.stamp;
}

void ScaleMap::ptamCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr posePtr)
{
  lastPtam = tf::Transform(tf::Quaternion(posePtr->pose.pose.orientation.x,posePtr->pose.pose.orientation.y,posePtr->pose.pose.orientation.z,posePtr->pose.pose.orientation.w),tf::Vector3(posePtr->pose.pose.position.x,posePtr->pose.pose.position.y,posePtr->pose.pose.position.z));

  lastPtamStamp = posePtr->header.stamp;

  //ROS_INFO("PTAM up: %lf,%lf,%lf",lastPtam.getOrigin().x(),lastPtam.getOrigin().y(),lastPtam.getOrigin().z());
}

double ScaleMap::getScale()
{
  double curDist,initNav;

  tf::Transform initPtam,finalPtam;
  tf::StampedTransform droneToCam;

  while((ros::Time::now() - lastNavStamp > ros::Duration(0.5)) || (ros::Time::now() - lastPtamStamp > ros::Duration(0.5)))
    {
      ROS_INFO("Waiting to sync callbacks...");
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1));
    }

  initPtam = lastPtam;
  initNav = lastNav;
  curDist = 0;

  //ROS_INFO("initNav = %lf, initPtam = %lf,%lf,%lf",initNav,initPtam.getOrigin().x(),initPtam.getOrigin().y(),initPtam.getOrigin().z());

  while(std::abs(curDist) < dist)
    {
      if(axis !=3)
	{
	  //ROS_INFO("Vel=%lf for %lf s",lastNav,(ros::Time::now()-lastNavStamp).toSec());
	  curDist += lastNav * ((ros::Time::now()-lastNavStamp).toSec());
	}
      else
	curDist = lastNav - initNav;

      ros::spinOnce();
    }
  ROS_INFO("Distance of %lf on inertial",curDist);

  finalPtam = lastPtam;
  //ROS_INFO("Final ptam = %lf,%lf,%lf",finalPtam.getOrigin().x(),finalPtam.getOrigin().y(),finalPtam.getOrigin().z());
  try
    {
      listener->waitForTransform("/ardrone_base_link","/ardrone_base_frontcam",lastPtamStamp,ros::Duration(1));
      listener->lookupTransform("/ardrone_base_link","/ardrone_base_frontcam",lastPtamStamp,droneToCam);
    }
  catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

  tf::Transform initToFinal = (droneToCam * initPtam) * ((droneToCam * finalPtam).inverse());

  double ptamDist = axis == 1 ? initToFinal.getOrigin().x() : axis == 2? initToFinal.getOrigin().y() : initToFinal.getOrigin().z();

  ROS_INFO("initToFinal: %lf,%lf,%lf, ptamDist: %lf",initToFinal.getOrigin().x(),initToFinal.getOrigin().y(),initToFinal.getOrigin().z(),ptamDist);

  if((ros::Time::now() - lastNavStamp < ros::Duration(0.5)) && (ros::Time::now() - lastPtamStamp < ros::Duration(0.5)))
    {
      if( curDist * ptamDist < 0)
	{
	  ROS_INFO("Sensors report movement in opposite directions!");
	  return -1;
	}
      else
	return (curDist/ptamDist);
    }
  else
    {
      ROS_INFO("Something went wrong in syncing the sensors, try scaling again...!");
      return -1;
    }

}
