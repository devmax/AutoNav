#include "Circle.h"

double angleFromTo(double angle, double min, double max)
{//make sure min<=angle<max
  while(angle<min) angle+=360;
  while(angle>=max) angle-=360;
  return angle;
}

Circle::Circle()
{
  vel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  log_control = n.advertise<AutoNav::circle_control>("/log_circleControl",1);
  state = n.subscribe("/ardrone/predictedPose",2,&Circle::stateCB,this);

  radiusInit = false;
  stateInit = false;

  iTerm = lastError = Eigen::Vector3f::Zero();
  lastTime = ros::Time::now();
}

void Circle::dynConfCB(AutoNav::CircleParamsConfig &config,uint32_t level)
{
  radius = config.radius;
  latVel = config.latVel;
  dirn = config.direction;

  latVel *= dirn;

  ROS_INFO("Target latVel: %lf, radius: %lf",latVel,radius);


  atr.Kp = config.Kat_p;
  atr.Kd = config.Kat_d;
  atr.Ki = config.Kat_i;

  ctr.Kp = config.Kct_p;
  ctr.Kd = config.Kct_d;
  ctr.Ki = config.Kct_i;

  angular.Kp = config.angular_p;
  angular.Kd = config.angular_d;
  angular.Ki = config.angular_i;
}

void Circle::stateCB(const AutoNav::filter_stateConstPtr state)
{

  if(!stateInit)
    {//store initial state and offset all goal commands by it
      initX = state->x;
      initY = state->y;
      initA = state->yaw;
      stateInit = true;
    }

  AutoNav::circle_control log_circle; //log all data for debugging

  Eigen::Vector2f error;
  Eigen::Vector3f d_error;
  Eigen::Vector2f proj_error,proj_vel;
  Eigen::Vector2f goal;

  double yawRad = state->yaw - initA;
  yawRad = angleFromTo(yawRad,-180,180) * PI / 180;

  //goal on the x and y axes in world frame
  goal(0) = initX + (radius*(1 - cos(yawRad)));
  goal(1) = initY - (radius*sin(yawRad));

  //error in position in world frame
  error(0) = goal(0) - state->x;
  error(1) = goal(1) - state->y;

  //error in position projected in drone's frame
  proj_error(0) = error(0)*cos(yawRad) + error(1)*sin(yawRad);
  proj_error(1) = -error(0)*sin(yawRad) + error(1)*cos(yawRad);

  //drone's velocity projected in its own frame
  proj_vel(0) = (state->dx)*cos(yawRad) + (state->dy)*sin(yawRad);
  proj_vel(1) = -(state->dx)*sin(yawRad) + (state->dy)*cos(yawRad);

  //desired angular velocity at this point
  angVel = (-proj_vel(1)) / radius;

  //error in velocities
  d_error(0) = - proj_vel(0);//should be zero in the cross track axis
  d_error(1) = latVel - proj_vel(1); //should be constant in the axis tangential to circle
  d_error(2) = angVel - ((state->dyaw)*PI/180); //angular velocity

  iTerm(0) = 0; //drone's x-axis
  iTerm(1) += lastError(1) * (ros::Time::now()-lastTime).toSec(); //drone's y-axis
  iTerm(2) += lastError(2) * (ros::Time::now()-lastTime).toSec(); //yaw 

  //cross track (normal to tangent of circle)
  double CTgainP = proj_error(0)*ctr.Kp; //proportional
  double CTgainD = d_error(0)*ctr.Kd; //derivative

  //along track
  double ATgainP = d_error(1)*atr.Kp; //proportional
  double ATgainI = iTerm(1) * atr.Ki; //integral

  double ANGgainP = d_error(2)*angular.Kp; //prop.
  double ANGgainI = iTerm(2) * angular.Ki; //integral

  lastError(1) = d_error(1);
  lastError(2) = d_error(2);
  lastTime = ros::Time::now();

  geometry_msgs::Twist cmd;

  cmd.linear.x = CTgainP + CTgainD;
  cmd.linear.y = ATgainP + ATgainI;
  cmd.angular.z = ANGgainP + ANGgainI;

  //START LOGGING

  log_circle.yaw = state->yaw - initA;
  log_circle.dyaw = state->dyaw;

  log_circle.angVel = angVel * 180/PI;

  log_circle.goalX = goal(0);
  log_circle.goalY = goal(1);

  log_circle.errorX = error(0);
  log_circle.errorY = error(1);

  log_circle.PerrorX = proj_error(0);
  log_circle.PerrorY = proj_error(1);

  log_circle.PvelX = proj_vel(0);
  log_circle.PvelY = proj_vel(1);

  log_circle.VerrX = d_error(0);
  log_circle.VerrY = d_error(1);
  log_circle.VerrA = d_error(2) * 180/PI;

  log_circle.CTgainP = CTgainP;
  log_circle.CTgainD = CTgainD;

  log_circle.ATgainP = ATgainP;
  log_circle.ATgainI = ATgainI;

  log_circle.ANGgainP = ANGgainP;
  log_circle.ANGgainI = ANGgainI;

  log_control.publish(log_circle);
  //END LOGGING

  //  ROS_INFO("Velocity commands are: %lf,%lf,%lf",cmd.linear.x,cmd.linear.y,cmd.angular.z);
  vel.publish(cmd);

}

void Circle::begin()
{
  while(n.ok())
    {
      while(!radiusInit)
	{ //set radius by observing the depth of the currently visible tag...
	  const ar_track_alvar::AlvarMarkersConstPtr msg = ros::topic::waitForMessage<ar_track_alvar::AlvarMarkers>("/ar_pose_marker",ros::Duration(5));
	  if(msg)
	    {
	      for(size_t i=0; i<msg->markers.size() && (!radiusInit); i++)
		{
		  if(msg->markers[i].id == 0)
		    {
		      radius = msg->markers[i].pose.pose.position.x + 0.3;
		      radiusInit = true;
		    }
		}
	      if(radiusInit)
		ROS_INFO("Target radius chosen to be %lf metres",radius);
	      else
		ROS_INFO("First tag not sighted, retrying callback..."); //always looks for tag ID 0, make sure that's where you start
	    }
	  else
	    ROS_INFO("No message received from tag callback...");
	}

      ros::spin();
    }
}
