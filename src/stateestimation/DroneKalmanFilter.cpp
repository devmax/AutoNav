#include "DroneKalmanFilter.h"
#include "EstimationNode.h"
#include<math.h>

// constants (variances assumed to be present)
const double varSpeedObservation_xy = 1*1;
const double varPoseObservation_xy = 0.3*0.3;
const double varAccelerationError_xy = 8*8;

const double varPoseObservation_z_tag = 0.35*0.35;
const double varPoseObservation_z_IMU = 0.15*0.15;
const double varPoseObservation_z_IMU_NO_tag = 0.1*0.1;
const double varAccelerationError_z = 1*1;

const double varPoseObservation_rp_tag = 8*8;
const double varPoseObservation_rp_IMU = 1*1;
const double varSpeedError_rp = 360*360 * 16;	// increased because prediction based on control command is very inaccurate.

const double varSpeedObservation_yaw = 3*3;
const double varPoseObservation_yaw_IMU =2*2;
const double varPoseObservation_yaw_tag = 4*4; //very inaccurate angle observations due to motion blur with the tag
const double varAccelerationError_yaw = 360*360;
	

// constants (assumed delays in ms).
// default ping values: nav=25, vid=50
int DroneKalmanFilter::delayRPY = 0;		// always zero
int DroneKalmanFilter::delayXYZ = 40;		// base_delayXYZ, but at most delayVideo
int DroneKalmanFilter::delayVideo = 75;		// base_delayVideo + delayVid - delayNav
int DroneKalmanFilter::delayControl = 80;	// base_delayControl + 2*delayNav

const int DroneKalmanFilter::base_delayXYZ = 40;		// t_xyz - t_rpy = base_delayXYZ
const int DroneKalmanFilter::base_delayVideo = 50;		// t_cam - t_rpy = base_delayVideo + delayVid - delayNav
const int DroneKalmanFilter::base_delayControl = 50;	// t_control + t_rpy - 2*delayNav

using namespace std;

double angleFromTo(double angle, double min, double max)
{//make sure min<=angle<max
  while(angle<min) angle+=360;
  while(angle>=max) angle-=360;
  return angle;
}

pthread_mutex_t DroneKalmanFilter::filter_CS = PTHREAD_MUTEX_INITIALIZER;

DroneKalmanFilter::DroneKalmanFilter()
{
  navdataQueue = new std::deque<ardrone_autonomy::Navdata>();
  velQueue = new std::deque<geometry_msgs::TwistStamped>();

  predictInternal_channel=n.resolveName("/log_predictInternal");
  predictUpTo_channel=n.resolveName("/log_predictUpTo");
  obs_IMU_XYZ_channel=n.resolveName("/log_obs_IMU_XYZ");
  obs_IMU_RPY_channel=n.resolveName("/log_obs_IMU_RPY");
  obs_PTAM_channel=n.resolveName("/log_obs_PTAM");
  predictData_channel=n.resolveName("/log_predictData");

  pub_predictInternal=n.advertise<AutoNav::predictInternal>(predictInternal_channel,1);
  pub_predictUpTo=n.advertise<AutoNav::predictUpTo>(predictUpTo_channel,1);
  pub_obs_IMU_XYZ=n.advertise<AutoNav::obs_IMU_XYZ>(obs_IMU_XYZ_channel,1);
  pub_obs_IMU_RPY=n.advertise<AutoNav::obs_IMU_RPY>(obs_IMU_RPY_channel,1);
  pub_obs_PTAM=n.advertise<AutoNav::obs_PTAM>(obs_PTAM_channel,1);

  last_z_IMU = -111;

  pthread_mutex_lock(&filter_CS);
  reset();
  pthread_mutex_unlock(&filter_CS);

  c1=0.58;
  c2=17.8;
  c3=10;
  c4=35;
  c5=10;
  c6=25;
  c7=1.4;
  c8=1.0;

}

void DroneKalmanFilter::release()
{
  delete navdataQueue;
  delete velQueue;
}

void DroneKalmanFilter::setPing(unsigned int navPing, unsigned int vidPing)
{
  //for now, ping values are constant...they should be periodically updated as in tum_ardrone
  // add a constant of 20ms // 40ms to accound for delay due to ros.
  // very, very rough approximation.
  navPing += 20;
  vidPing += 40;

  int new_delayXYZ = base_delayXYZ;
  int new_delayVideo = base_delayVideo + vidPing/(int)2 - navPing/(int)2;
  int new_delayControl = base_delayControl + navPing;
	
  delayXYZ = std::min(500,std::max(40,std::min(new_delayVideo,new_delayXYZ)));
  delayVideo = std::min(500,std::max(40,new_delayVideo));
  delayControl = std::min(200,std::max(50,new_delayControl));

  std::cout << "new delasXYZ: " << delayXYZ << ", delayVideo: " << delayVideo << ", delayControl: " << delayControl << std::endl;
}

void DroneKalmanFilter::reset()
{
  ROS_INFO("Doing a reset on EKF now!");
  // init filter with pose 0 (var 0) and speed 0 (var large).

  x = y = z = PVSFilter(0,1);

  yaw = PVFilter(0);
  roll = pitch = PFilter(0);
  
  last_z_heightDiff = 0;

  clearPTAM();

  // clear IMU-queues
  navdataQueue->clear();
  velQueue->clear();
	
  predictedUpToTimestamp = getMS(ros::Time::now());
  predictedUpToTotal = -1;

  baselineZ_Filter = baselineZ_IMU = baselineY_IMU = baselineY_Filter = -999999;

  yaw_offset_initialized = false;
}

void DroneKalmanFilter::resetPoseVariances()
{
  ROS_INFO("Setting state back to origin..");

  x.state(0) = x.state(1) = 0;
  y.state(0) = y.state(1) = 0;
  z.state(0) = z.state(1) = 0;

  x.var = y.var = z.var = Eigen::Matrix3f::Zero();
}

void DroneKalmanFilter::clearPTAM()
{
  ROS_INFO("Initializing PTAM-info now!");

  last_yaw =  last_z = 0.0;
  last_PTAM = getMS(ros::Time::now());

  lastPosesValid = last_yaw_valid = false;
}

void DroneKalmanFilter::predictInternal(geometry_msgs::Twist activeControlInfo, int timeSpanMicros, bool useControlGains)
{
  if(timeSpanMicros <= 0) return;

  bool controlValid = !(activeControlInfo.linear.z > 1.01 || activeControlInfo.linear.z < -1.01 || activeControlInfo.linear.x > 1.01 || activeControlInfo.linear.x < -1.01 || activeControlInfo.linear.y > 1.01 || activeControlInfo.linear.y < -1.01 || activeControlInfo.angular.z > 1.01 || activeControlInfo.angular.z < -1.01);

  double tsMillis = timeSpanMicros / 1000.0;	// in milliseconds
  double tsSeconds = tsMillis / 1000.0;	// in seconds

  //ROS_INFO("Predicting based on :(%lf,%lf,%lf) and %lf",activeControlInfo.linear.x,activeControlInfo.linear.y,activeControlInfo.linear.z,activeControlInfo.angular.z);

  // predict roll, pitch, yaw
  float rollControlGain = tsSeconds*c3*(c4 * std::max(-0.5, std::min(0.5, -(double)activeControlInfo.linear.y)) - roll.state);
  float pitchControlGain = tsSeconds*c3*(c4 * std::max(-0.5, std::min(0.5, (double)activeControlInfo.linear.x)) - pitch.state);
  float yawSpeedControlGain = tsSeconds*c5*(c6 * activeControlInfo.angular.z - yaw.state[1]);

  //ROS_INFO("Roll gain: %lf tending to %lf = %lf",roll.state,-activeControlInfo.linear.y,rollControlGain);
  //ROS_INFO("Pitch gain: %lf tending to %lf = %lf",pitch.state,activeControlInfo.linear.x,pitchControlGain);

  double yawRad = yaw.state[0] * 3.14159268 / 180;
  double rollRad = roll.state * 3.14159268 / 180;
  double pitchRad = pitch.state * 3.14159268 / 180;
  //double forceX = cos(yawRad) * sin(rollRad) * cos(pitchRad) - sin(yawRad) * sin(pitchRad);
  //double forceY = - sin(yawRad) * sin(rollRad) * cos(pitchRad) - cos(yawRad) * sin(pitchRad);
	
  double forceX = cos(rollRad)*sin(pitchRad)*cos(yawRad) + sin(rollRad)*sin(yawRad);
  double forceY = cos(rollRad)*sin(pitchRad)*sin(yawRad) - sin(rollRad)*cos(yawRad);

  double vx_gain = tsSeconds * c1 * (c2*forceX - x.state(1));
  double vy_gain = tsSeconds * c1 * (c2*forceY - y.state(1));
  double vz_gain = tsSeconds * c7 * (c8*activeControlInfo.linear.z*(activeControlInfo.linear.z < 0 ? 2 : 1) - z.state[1]);

  //ROS_INFO("forceX = %lf, vx = %lf and vx_gain = %lf",forceX,x.state(1),vx_gain);
  //ROS_INFO("forceY = %lf, vy = %lf and vy_gain = %lf",forceY,y.state(1),vy_gain);

  /*begin LOGGING*/
  AutoNav::predictInternal message;
  message.timestamp = getMS();
  message.timeSpanMicros = timeSpanMicros;
  message.activeControlInfo = activeControlInfo;
  message.useControlGains = (int)useControlGains;
  message.controlValid = (int)controlValid;
  message.rollControlGain = rollControlGain;
  message.pitchControlGain = pitchControlGain;
  message.yawSpeedControlGain = yawSpeedControlGain;
  message.forceX = forceX;
  message.forceY = forceY;
  message.vx_gain = vx_gain;
  message.vy_gain = vy_gain;
  message.vz_gain = vz_gain;

  message.prior_state = getCurrentState();
  message.prior_var = getCurrentVariances();

  /*end LOGGING*/

  if(!useControlGains || !controlValid)
    {
      //ROS_INFO("useControlGains = %d and controlValid = %d",(int)useControlGains,(int)controlValid);
      vx_gain = vy_gain = vz_gain = 0;
      rollControlGain = pitchControlGain = yawSpeedControlGain = 0;
    }

  //propagate state
  yaw.state[0]=angleFromTo(yaw.state[0],-180,180);
  roll.predict(tsMillis,varSpeedError_rp, rollControlGain);
  pitch.predict(tsMillis,varSpeedError_rp, pitchControlGain);
  yaw.predict(tsMillis,varAccelerationError_yaw,(Eigen::Vector2f()<<(tsSeconds*yawSpeedControlGain/2),yawSpeedControlGain).finished(),1,5*5);
  yaw.state[0]=angleFromTo(yaw.state[0],-180,180);

  x.predict(tsMillis,varAccelerationError_xy,(Eigen::Vector3f()<<(tsSeconds*vx_gain/2),vx_gain,0).finished(),0.0001);
  y.predict(tsMillis,varAccelerationError_xy,(Eigen::Vector3f()<<(tsSeconds*vy_gain/2),vy_gain,0).finished(),0.0001);
  //CONTACT ABOUT BUG HERE-VARIANCE IN SPEED SENT IMPROPER
  z.predict(tsMillis,(Eigen::Vector3f()<<(tsSeconds*tsSeconds*tsSeconds*tsSeconds), (9*tsSeconds*tsSeconds),(tsSeconds*tsSeconds*tsSeconds*3)).finished(),(Eigen::Vector3f()<<(tsSeconds*vz_gain/2),vz_gain,0).finished());

  /*begin LOGGING*/
  message.posterior_state = getCurrentState();

  message.posterior_var = getCurrentVariances();

  pub_predictInternal.publish(message);
  /*end LOGGING*/
}

void DroneKalmanFilter::observeIMU_XYZ(const ardrone_autonomy::Navdata* nav)
{
  //ROS_INFO("Observing IMU_XYZ # %d now!",nav->header.seq);
  double yawRad = yaw.state[0]*3.14159268 / 180;
  double vx_global = (cos(yawRad)*nav->vx - sin(yawRad)*nav->vy)/1000.0;
  double vy_global = (sin(yawRad)*nav->vx + cos(yawRad)*nav->vy)/1000.0;

  /*begin LOGGING*/
  AutoNav::obs_IMU_XYZ message;
  message.timestamp=getMS();
  message.seq = nav->header.seq;
  message.nav_vx = nav->vx;
  message.nav_vy = nav->vy;
  message.global_vx = vx_global;
  message.global_vy = vy_global;

  message.pre_state = getCurrentState();
  message.pre_var = getCurrentVariances();

  /*end LOGGING*/

  if(lastPosesValid)
    {
      x.observeSpeed(vx_global,varSpeedObservation_xy*20);//50);
      y.observeSpeed(vy_global,varSpeedObservation_xy*20);//50);
    }
  else
    {
      x.observeSpeed(vx_global,varSpeedObservation_xy);
      y.observeSpeed(vy_global,varSpeedObservation_xy);
    }

  if(last_z_IMU != nav->altd || nav->header.seq - last_z_packageID>6)
    {
      //altimeter is used as a relative height sensor, not absolute...
      if(baselineZ_Filter < -100000)
	{
	  baselineZ_IMU = nav->altd;
	  baselineZ_Filter = z.state[0];
	}
      if(lastPosesValid)
	{
	  double imuHeightDiff = (nav->altd - baselineZ_IMU)*0.001;
	  double observedHeight = baselineZ_Filter + 0.5*(imuHeightDiff+last_z_heightDiff);
	  last_z_heightDiff = imuHeightDiff;

	  baselineZ_IMU = nav->altd;
	  baselineZ_Filter = z.state[0];

	  if((abs(imuHeightDiff)<0.150 && abs(last_z_heightDiff)<0.150))
	    {
	      z.observePose(observedHeight,varPoseObservation_z_IMU);
	    }
	}
      else
	{
	  double imuHeightDiff=(nav->altd - baselineZ_IMU)*0.001;
	  double observedHeight=baselineZ_Filter + imuHeightDiff;

	  if(abs(imuHeightDiff)<0.110)
	    {
	      z.observePose(observedHeight,varPoseObservation_z_IMU_NO_tag);
	    }
	  else
	    {
	      if(baselineZ_IMU == 0 || nav->altd == 0)
		{
		  z.observePose(observedHeight,0);
		  z.observeSpeed(0,0);
		}
	      baselineZ_IMU = nav->altd;
	      baselineZ_Filter = z.state[0];
	    }
	}
      last_z_IMU = nav->altd;
      last_z_packageID = nav->header.seq;
    }
  /*begin LOGGING*/

  message.post_state = getCurrentState();
  message.post_var = getCurrentVariances();

  pub_obs_IMU_XYZ.publish(message);
  /*end LOGGING*/
}

void DroneKalmanFilter::observeIMU_RPY(const ardrone_autonomy::Navdata* nav)
{
  /*begin LOGGING*/
  AutoNav::obs_IMU_RPY message;
  message.timestamp=getMS();
  message.seq = nav->header.seq;
  message.roll = nav->rotX;
  message.pitch = nav->rotY;

  message.pre_state = getCurrentState();
  message.pre_var = getCurrentVariances();
  /*end LOGGING*/

  roll.observe(nav->rotX,varPoseObservation_rp_IMU);
  pitch.observe(nav->rotY,varPoseObservation_rp_IMU);

  if(baselineY_Filter<-100000)
    {
      baselineY_IMU = nav->rotZ;
      baselineY_Filter = yaw.state[0];

      //ROS_INFO("Initing: base_IMU = %lf and base_Filter = %lf",baselineY_IMU,baselineY_Filter);
    }

  double imuYawDiff = (nav->rotZ - baselineY_IMU);
  double observedYaw = baselineY_Filter + imuYawDiff;

  //ROS_INFO("imuYawDiff=%lf and observedYaw=%lf",imuYawDiff,observedYaw);

  yaw.state[0] = angleFromTo(yaw.state[0],-180,180);
  observedYaw = angleFromTo(observedYaw,-180,180);


  if(last_yaw_valid)
    {
      //yaw measurements are also treated as relative to last measurement, not absolute...
      baselineY_IMU = nav->rotZ;
      baselineY_Filter = yaw.state[0];


      if(abs(observedYaw - yaw.state[0])<15)
	{
	  yaw.observePose(observedYaw,varPoseObservation_yaw_IMU);
	  message.code = 1;
	  //ROS_INFO("New yaw after observation = %lf",yaw.state(0));
	}
      else
	{
	  //ROS_INFO("Large jump in NAVdata yaw, rejecting!");
	  message.code = 2;
	}
    }
  else
    {
      if(abs(observedYaw - yaw.state[0])<25)
	{
	  //ROS_INFO("Prior yaw: %lf",yaw.state(0));
	  yaw.observePose(observedYaw,1*1);
	  message.code = 3;
	  //ROS_INFO("Last pose invalid, but making observation now! posterior=%lf",yaw.state(0));
	}
      else
	{
	  //ROS_INFO("Last tag observation invalid and big jump in NAVyaw!");
	  baselineY_IMU=nav->rotZ;
	  baselineY_Filter = yaw.state(0);
	  message.code = 4;
	  //ROS_INFO("invalid: baselineY_IMU=%lf,baselineY_Filter=%lf",baselineY_IMU,baselineY_Filter);
	}
    }

  message.baselineY_IMU = baselineY_IMU;
  message.baselineY_Filter = baselineY_Filter;
  message.navYaw = nav->rotZ;
  message.observedYaw = observedYaw;

  yaw.state[0]=angleFromTo(yaw.state[0],-180,180);

  /*begin LOGGING*/

  message.post_state = getCurrentState();
  message.post_var = getCurrentVariances();

  pub_obs_IMU_RPY.publish(message);
  /*end LOGGING*/
}

void DroneKalmanFilter::observePTAM(Vector6f pose,Vector6f var)
{
  /*LOGGING now*/
  AutoNav::obs_PTAM message;

  message.timestamp = getMS();
  message.x = pose(0);
  message.y = pose(1);
  message.z = pose(2);
  message.roll = pose(3);
  message.pitch = pose(4);
  message.yaw = pose(5);

  message.varX = var(0);
  message.varY = var(1);
  message.varZ = var(2);
  message.varRoll = var(3);
  message.varPitch = var(4);
  message.varYaw = var(5);

  message.prior_state = getCurrentState();
  message.prior_var = getCurrentVariances();

  /*end LOGGING*/

  bool reject  = false;
  lastPosesValid = true;

  x.observePose(pose(0),var(0));
  y.observePose(pose(1),var(1));
  z.observePose(pose(2),var(2));

  //x.observePose(pose[0],varPoseObservation_xy);
  //y.observePose(pose[1],varPoseObservation_xy);

  //  if(std::abs(last_z-pose(2))<2 || (getMS()-last_tag)>500)
  //    {
  //z.observePose(pose[2],varPoseObservation_z_tag);
  //    }
  //  else
  //ROS_INFO("Large jump in z, not observing height..");

  //ROS_INFO("Posterior poses: %lf,%lf,%lf and velocities: %lf,%lf,%lf",x.state(0),y.state(0),z.state(0),x.state(1),y.state(1),z.state(1));

  roll.observe(pose(3),var(3));
  pitch.observe(pose(4),var(4));

  //roll.observe(pose[3],varPoseObservation_rp_tag);
  //pitch.observe(pose[4],varPoseObservation_rp_tag);

  pose[5] = angleFromTo(pose[5],-180,180);

  //yaw.observePose(pose(5),varPoseObservation_yaw_tag);
  yaw.observePose(pose(5),var(5));

  yaw.state[0] = angleFromTo(yaw.state[0],-180,180);


  if(abs(x.state(1))>2.8)
    {
      //      ROS_INFO("Large jump in X of %lf, rejecting observation!",x.state(1));
      reject = true;
    }
  else if(abs(y.state(1))>2.8)
    {
      //      ROS_INFO("Large jump in Y of %lf, rejecting observation!",y.state(1));
      reject = true;
    }
  else if(abs(z.state(1))>4.0)
    {
      //      ROS_INFO("Large jump in Z of %lf!",z.state(1));
      reject = true;
    }

  /*begin LOGGING*/
  message.posterior_state = getCurrentState();
  message.posterior_var = getCurrentVariances();

  message.rejected = (int)reject;
  pub_obs_PTAM.publish(message);
  /*end LOGGING*/
}

void DroneKalmanFilter::predictUpTo(int timestamp, bool consume, bool useControlGains)
{
  if(predictedUpToTimestamp == timestamp)
    return;

  //ROS_INFO("PredictUpTo %d to %d",predictedUpToTimestamp,timestamp);
  //std::cout << (consume ? "per" : "tmp") << " pred @ " << this << ": " << predictedUpToTimestamp << " to " << timestamp << std::endl;

  // at this point:
  // - velQueue contains controls, timestamped with time at which they were sent.
  // - navQueue contains navdata, timestamped with time at which they were received.
  // - timestamp is the time up to which we want to predict, i.e. maybe a little bit into the future

  // start at [predictdUpToTimestamp]. predict step-by-step observing at [currentTimestamp] the
  // - rpy timestamped with [currentTimestamp + delayRPY]
  // - xyz timestamped with [currentTimestamp + delayXYZ]
  // using
  // - control timestamped with [currentTimestamp - delayControl]

  // fast forward until first package that will be used.
  // for controlIterator, this is the last package with a stamp smaller/equal than what it should be.
  // for both others, this is the first package with a stamp bigger than what it should be.
  // if consume, delete everything before permanently.
  std::deque<geometry_msgs::TwistStamped>::iterator controlIterator = velQueue->begin();

  while(controlIterator != velQueue->end() && controlIterator+1 != velQueue->end() && getMS((controlIterator+1)->header.stamp) <= predictedUpToTimestamp - delayControl)
    {
      if(consume)
	{
	  velQueue->pop_front();
	  controlIterator = velQueue->begin();
	}
      else
	controlIterator++;
    }
  if(velQueue->size() == 0)
    {
      //ROS_INFO("VelQueue = 0!");
      useControlGains = false;
    }
  // dont delete here, it will be deleted if respective rpy data is consumed.
  std::deque<ardrone_autonomy::Navdata>::iterator xyzIterator = navdataQueue->begin();
  while(xyzIterator != navdataQueue->end() && getMS(xyzIterator->header.stamp) <= predictedUpToTimestamp + delayXYZ)
    {
      xyzIterator++;
    }

  std::deque<ardrone_autonomy::Navdata>::iterator rpyIterator = navdataQueue->begin();
  while(rpyIterator != navdataQueue->end() && getMS(rpyIterator->header.stamp) <= predictedUpToTimestamp + delayRPY)
    {
      if(consume)
	{
	  //ROS_INFO("Popping Navdata message # %d ",(navdataQueue->front()).header.seq);
	  navdataQueue->pop_front();
	  rpyIterator = navdataQueue->begin();
	}
      else
	rpyIterator++;
    }
  // now, each iterator points to the first element in queue that is to be integrated.
  // start predicting,
  while(true)
    {
      // predict ahead to [timestamp]
      int predictTo = timestamp;
      
      // but a maximum of 10ms per prediction step, to guarantee nonlinearities.
      predictTo =std::min(predictTo, predictedUpToTimestamp+10);

      // get three queues to the right point in time by rolling forward in them.
      // for xyz this is the first point at which its obs-time is bigger than or equal to [predictedUpToTimestamp]
      while(xyzIterator != navdataQueue->end() && getMS(xyzIterator->header.stamp) - delayXYZ < predictedUpToTimestamp)
	{
	  xyzIterator++;
	}
      while(rpyIterator != navdataQueue->end() && getMS(rpyIterator->header.stamp) - delayRPY < predictedUpToTimestamp)
	{
	  rpyIterator++;
	}
      // for control that is last message with stamp <= predictedUpToTimestamp - delayControl.
      while(controlIterator != velQueue->end() && controlIterator+1 != velQueue->end() && getMS((controlIterator+1)->header.stamp) + delayControl <= predictedUpToTimestamp)
	{
	  controlIterator++;
	}
      /*begin LOGGING*/
      AutoNav::predictUpTo message;
      message.timestamp = getMS();
      message.controlInfo = useControlGains?controlIterator->twist:geometry_msgs::Twist();
      message.consume = (int)consume; 
      message.age = predictedUpToTimestamp - delayControl - getMS(controlIterator->header.stamp);
      /*end LOGGING*/

      // predict not further than the point in time where the next observation needs to be added.

      if(rpyIterator != navdataQueue->end() )
	predictTo =std::min(predictTo, getMS(rpyIterator->header.stamp)-delayRPY);
      if(xyzIterator != navdataQueue->end() )
	predictTo = std::min(predictTo,getMS(xyzIterator->header.stamp)-delayXYZ);
      

      predictInternal(useControlGains ? controlIterator->twist : geometry_msgs::Twist(),(predictTo - predictedUpToTimestamp)*1000,useControlGains && getMS(controlIterator->header.stamp) + 200 > predictedUpToTimestamp - delayControl);				// control max. 200ms old.


      // if an observation needs to be added, it HAS to have a stamp equal to [predictTo],
      // as we just set [predictTo] to that timestamp.
      bool observedXYZ = false, observedRPY=false;
      if(rpyIterator != navdataQueue->end() && getMS(rpyIterator->header.stamp) - delayRPY == predictTo)
	{
	  observeIMU_RPY(&(*rpyIterator));
	  /*begin LOGGING*/
	  message.seq_rpy = rpyIterator->header.seq;
	  message.roll = rpyIterator->rotX;
	  message.pitch = rpyIterator->rotY;
	  message.yaw = rpyIterator->rotZ;
	  /*end LOGGING*/

	  observedRPY = true;
	}
      if(xyzIterator != navdataQueue->end() && getMS(xyzIterator->header.stamp)-delayXYZ == predictTo)
	{
	  //ROS_INFO("Odometer data # %d received at %d.%d=%d selected for observation!",xyzIterator->header.seq,xyzIterator->header.stamp.sec,xyzIterator->header.stamp.nsec,getMS(xyzIterator->header.stamp));
	  //if(this->useNavdata)
	  observeIMU_XYZ(&(*xyzIterator));
	  /*begin LOGGING*/
	  message.seq_xyz = xyzIterator->header.seq;
	  message.vx = xyzIterator->vx;
	  message.vy = xyzIterator->vy;
	  message.altd = xyzIterator->altd;
	  /*end LOGGING*/
	  observedXYZ = true;
	}

      predictedUpToTimestamp = predictTo;

      if(consume)
	predictedUpToTotal = predictedUpToTimestamp;

      if(observedRPY) 
	rpyIterator++;
      if(observedXYZ) 
	xyzIterator++;

      /*begin LOGGING*/
      pub_predictUpTo.publish(message);
      /*end LOGGING*/

      // if this is where we wanna get, quit.
      if(predictTo == timestamp)
	break;

      /*begin LOGGING*/
      message.seq_rpy = -1;
      message.seq_xyz = -1;
      message.roll = 0.0;
      message.pitch = 0.0;
      message.yaw = 0.0;
      message.vx = 0.0;
      message.vy = 0.0;
      message.altd = 0.0;
      /*end LOGGING*/
    }
}

tf::Transform DroneKalmanFilter::getCurrentTF()
{
  return tf::Transform(tf::createQuaternionFromRPY(roll.state*PI/180,pitch.state*PI/180,yaw.state(0)*PI/180),tf::Vector3(x.state(0),y.state(0),z.state(0)));
}
Vector6f DroneKalmanFilter::getCurrentPose()
{
  return (Vector6f()<<x.state[0],y.state[0],z.state[0],roll.state,pitch.state,yaw.state[0]).finished();
}

AutoNav::filter_state DroneKalmanFilter::getCurrentState()
{
  AutoNav::filter_state s;

  s.x = x.state(0);
  s.y = y.state(0);
  s.z = z.state(0);
  s.Lx = x.state(2);
  s.Ly = y.state(2);
  s.Lz = z.state(2);
  s.dx = x.state(1);
  s.dy = y.state(1);
  s.dz = z.state(1);
  s.roll = roll.state;
  s.pitch = pitch.state;
  s.yaw = yaw.state(0);
  s.dyaw = yaw.state(1);

  return s;
}

AutoNav::filter_var DroneKalmanFilter::getCurrentVariances()
{
  AutoNav::filter_var var;

  var.x = x.var(0,0);
  var.y = y.var(0,0);
  var.z = z.var(0,0);

  var.dx = x.var(1,1);
  var.dy = y.var(1,1);
  var.dz = z.var(1,1);

  var.Lx = x.var(2,2);
  var.Ly = y.var(2,2);
  var.Lz = z.var(2,2);

  var.roll = roll.var;
  var.pitch = pitch.var;
  var.yaw = yaw.var(0,0);
  var.dyaw = yaw.var(1,1);

  return var;
}

Vector10f DroneKalmanFilter::getCurrentPoseSpeedVariances()
{
  return (Vector10f()<<x.var(0,0),y.var(0,0),z.var(0,0),roll.var,pitch.var,yaw.var(0,0),x.var(1,1),y.var(1,1),z.var(1,1),yaw.var(1,1)).finished();
}

Vector6f DroneKalmanFilter::getCurrentPoseVariances()
{
  return (Vector6f()<<x.var(0,0),y.var(0,0),z.var(0,0),roll.var,pitch.var,yaw.var(0,0)).finished();
}

void DroneKalmanFilter::setScale(double scale,int axis)
{
  switch(axis)
    {
    case 1:
      x.state(2) = scale;
      ROS_INFO("Scale on X-axis = %lf",x.state(2));
      break;
    case 2:
      y.state(2) = scale;
      ROS_INFO("Scale on Y-axis = %lf",y.state(2));
      break;
    case 3:
      z.state(2) = scale;
      ROS_INFO("Scale on Z-axis = %lf",z.state(2));
      break;
    }
}

void DroneKalmanFilter::addPTAM(Vector6f measurement,Vector6f var,int corrStamp)
{
  //ROS_INFO("Adding tag now!");
  if(corrStamp>predictedUpToTotal)
    {
      //ROS_INFO("Calling predictUpTo by adding new tag to predict until %d",corrStamp);
      predictUpTo(corrStamp,true,true);
    }

  observePTAM(measurement,var);
  last_yaw_valid = true;
  /*  if(std::abs(last_yaw-measurement[5])<25 || (getMS() - last_PTAM)>500)
    {
      last_PTAM = getMS();
      last_yaw_valid = true;
      last_yaw = measurement(5);

      observePTAM(measurement);

    }
  else
    {
      last_yaw_valid = false;
      ROS_INFO("Unusual yaw of %lf detected from tags, rejecting!",measurement(5));
    }
  */
}

void DroneKalmanFilter::addFakePTAM(int timestamp)
{
  last_yaw_valid = false;

  if(timestamp>predictedUpToTotal)
    {

      predictUpTo(timestamp,true,true);
    }

  lastPosesValid = false;
}

AutoNav::filter_state DroneKalmanFilter::getPoseAt(ros::Time t, bool useControlGains)
{
  // make shallow copy
  DroneKalmanFilter scopy = DroneKalmanFilter(*this);

  // predict using this copy
  scopy.predictUpTo(getMS(t),false, useControlGains);

  // return value, and discard any changes made to scopy (deleting it)
  return scopy.getCurrentState();
}
