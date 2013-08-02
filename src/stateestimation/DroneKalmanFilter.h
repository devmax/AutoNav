#pragma once

#ifndef _DRONEKALMANFILTER_H_
#define _DRONEKALMANFILTER_H_

#include<Eigen/Core>
#include<Eigen/LU>
#include<deque>
#include<ros/ros.h>
#include<iostream>
#include<ardrone_autonomy/Navdata.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/TwistStamped.h>
#include<pthread.h>
#include"AutoNav/filter_state.h"
#include"AutoNav/obs_IMU_RPY.h"
#include"AutoNav/obs_IMU_XYZ.h"
#include"AutoNav/obs_tag.h"
#include"AutoNav/offsets.h"
#include"AutoNav/predictInternal.h"
#include"AutoNav/predictUpTo.h"
#include<algorithm>
#include<geometry_msgs/Pose.h>
#include<tf/tfMessage.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>

#ifndef _EIGEN_TYPES_
#define _EIGEN_TYPES_
typedef Eigen::Matrix<float,6,1> Vector6f;
typedef Eigen::Matrix<float,10,1> Vector10f;

const double PI = 3.14159265359;
#endif //_EIGEN_TYPES_

class EstimationNode;

class PVFilter
{
 public:
  Eigen::Vector2f state;
  Eigen::Matrix2f var;	

  // constructors
  inline PVFilter(Eigen::Vector2f state, Eigen::Matrix2f var)
    : state(state), var(var)
  {
  }

  inline PVFilter(double pose, double speed):state((Eigen::Vector2f()<<pose,speed).finished()),var(Eigen::Matrix2f::Zero())
    {
    }

  inline PVFilter(double pose)
    : state((Eigen::Vector2f()<<pose,0).finished()), var(Eigen::Matrix2f::Zero())
    {
      var(1,1) = 1e10;
    }

  inline PVFilter()
    : state(Eigen::Vector2f::Zero()), var(Eigen::Matrix2f::Zero())
    {
      var(1,1) = var(0,0) = 1e10;
    }

  // observe
  inline void observePose(double obs, double obsVar)
  {
    /* MATLAB:
       H = [1 0];
       K = (uncertainty * H') / ((H * uncertainty * H') + obsVar);
       state = state + K * (obs - H*state);
       var = (eye(2)-K*H) * var;
    */
    Eigen::Vector2f K = var.col(0) / (obsVar + var(0,0));	//K is first col = first row of var.

    //ROS_INFO("PoseObservation:: Variance and covariance: [%lf,%lf]'",var(0,0),var(1,0));
    //ROS_INFO("Kalman gain: [%lf,%lf]'",K[0],K[1]);
    
    //Eigen::Vector2f gain = K * (obs - state[0]);
    state = state + K * (obs - state[0]);

    //ROS_INFO("State augmented by: [%lf,%lf]'",gain[0],gain[1]);

    Eigen::Matrix2f tmp = Eigen::Matrix2f::Identity();
    tmp(0,0) -= K[0]; 
    tmp(1,0) -= K[1];
    var = tmp * var;

  }


  inline void observeSpeed(double obs, double obsVar)
  {
    /* MATLAB:
       H = [0 1];
       K = (uncertainty * H') / ((H * uncertainty * H') + obsVar);
       state = state + K * (observation - H*state);
       uncertainty = (eye(2)-K*H) * uncertainty;
    */
    Eigen::Vector2f K = var.col(1) / (obsVar + var(1,1));	//K is second col = second row of var.
    //ROS_INFO("SpeedObervation:: Variance and covariance: [%lf,%lf]'",var(1,1),var(0,1));
    //ROS_INFO("Kalman gain: [%lf,%lf]'",K[0],K[1]);

    //Eigen::Vector2f gain = K * (obs - state[1]);
    state = state + K * (obs - state[1]);

    Eigen::Matrix2f tmp = Eigen::Matrix2f::Identity();
    
    //ROS_INFO("State augmented by: [%lf,%lf]'",gain[0],gain[1]);

    tmp(0,1) -= K[0]; 
    tmp(1,1) -= K[1];
    var = tmp * var;
  }


  // predict
  // calculates prediction variance matrix based on gaussian acceleration as error.
  inline void predict(double ms, double accelerationVar, Eigen::Vector2f controlGains =(Eigen::Vector2f()<<0,0).finished() , double coVarFac = 1, double speedVarFac = 1)
  {
    /* MATLAB:
       G = [1 ms/1000; 0 1];
       E = [((ms/1000)^2)/2; ms/1000]; % assume normal distributed constant ACCELERATION.
       state = G*state;
       var = G * var * G' + accelerationVarPerS*(E*E');
    */

    ms /= 1000;

    Eigen::Matrix2f G = Eigen::Matrix2f::Identity();
    G(0,1) = ms;

    state = G * state + controlGains;
    var  = G * var * G.transpose();
    var(0,0) += accelerationVar * 0.25 * ms*ms*ms*ms;
    var(1,0) += coVarFac * accelerationVar * 0.5 * ms*ms*ms * 4;
    var(0,1) += coVarFac * accelerationVar * 0.5 * ms*ms*ms * 4;
    var(1,1) += speedVarFac * accelerationVar * 1 * ms*ms * 4 * 4;
  }

  // predict
  // calculates prediction using the given uncertainty matrix
  // vars is var(0) var(1) covar(0,1)
  inline void predict(double ms, Eigen::Vector3f vars, Eigen::Vector2f controlGains = Eigen::Vector2f::Zero())
  {
    /* MATLAB:
       G = [1 ms/1000; 0 1];
       E = [((ms/1000)^2)/2; ms/1000]; % assume normal distributed constant ACCELERATION.
       state = G*state;
       var = G * var * G' + accelerationVarPerS*(E*E');
    */

    ms /= 1000;

    Eigen::Matrix2f G = Eigen::Matrix2f::Identity();
    G(0,1) = ms;

    state = G * state + controlGains;
    var  = G * var * G.transpose();
    var(0,0) += vars[0];
    var(1,0) += vars[2];
    var(0,1) += vars[2];
    var(1,1) += vars[1];
  }
};

// KalmanFilter with only one component (pose, is observed directly)
class PFilter
{
 public:
  double state;
  double var;
	
  inline PFilter() : state(0), var(1e10) {};
  inline PFilter(double initState) : state(initState), var(0) {};

  inline void predict(double ms, double speedVar, double controlGains = 0)
  {
    /* MATLAB:
       state = state;
       var = var + speedVar*((ms/1000)^2);
    */
    state += controlGains;
    var += speedVar * ms * ms / 1000000;
  }

  inline void observe(double obs, double obsVar)
  {
    /* MATLAB:
       obs_w = var / (var + obsVar);
       state = state * (1-obs_w) + obs * obs_w;
       var = var*obsVar / (var + obsVar);
    */
    double w = var / (var + obsVar);
    state = (1-w) * state + w * obs;
    var = var * obsVar / (var + obsVar);
  }
};

class DroneKalmanFilter
{
 private:
  PVFilter x;
  PVFilter y;
  PVFilter z;
  PVFilter yaw;
  PFilter roll;
  PFilter pitch;

  bool lastPosesValid,yaw_offset_initialized,last_yaw_valid;

  double last_z_IMU;
  long last_z_packageID;

  void predictInternal(geometry_msgs::Twist activeControlInfo, int timeSpanMicros, bool useControlGains=true);
  void observeIMU_XYZ(const ardrone_autonomy::Navdata* nav);
  void observeIMU_RPY(const ardrone_autonomy::Navdata* nav);
  void observeTag(Vector6f measurement);

  int predictedUpToTotal;

  double baselineZ_IMU;
  double baselineZ_Filter;
  double last_z_heightDiff;
  double baselineY_IMU;
  double baselineY_Filter;
  double last_yaw;
  double last_z;

  int last_tag;

  std::string predictInternal_channel;
  std::string predictData_channel;
  std::string predictUpTo_channel;
  std::string obs_IMU_XYZ_channel;
  std::string obs_IMU_RPY_channel;
  std::string obs_tag_channel;

  ros::NodeHandle n;

  ros::Publisher pub_predictInternal;
  ros::Publisher pub_predictData;
  ros::Publisher pub_predictUpTo;
  ros::Publisher pub_obs_IMU_XYZ;
  ros::Publisher pub_obs_IMU_RPY;
  ros::Publisher pub_obs_tag;

 public:
  DroneKalmanFilter();

  void release();

  static int delayRPY;
  static int delayXYZ;
  static int delayVideo;
  static int delayControl;
  static const int base_delayXYZ;
  static const int base_delayVideo;
  static const int base_delayControl;

  std::deque<ardrone_autonomy::Navdata>* navdataQueue;
  std::deque<geometry_msgs::TwistStamped>* velQueue;

  static pthread_mutex_t filter_CS;

  int predictedUpToTimestamp;

  void reset();
  void clearTag();
  void predictUpTo(int timestamp,bool consume=true,bool useControlGains=true);
  void setPing(unsigned int navPing, unsigned int vidPing);

  Vector6f getCurrentPose();
  tf::Transform getCurrentTF();
  AutoNav::filter_state getCurrentState();
  Vector10f getCurrentPoseSpeedVariances();
  Vector6f getCurrentPoseVariances();
  
  float c1,c2,c3,c4,c5,c6,c7,c8;

  void addTag(Vector6f measurement,int corrStamp);
  void addFakeTag(int timestamp);

  AutoNav::filter_state getPoseAt(ros::Time t,bool useControlGains=true);
  
};
#endif  //DRONEKALMANFILTER.H
