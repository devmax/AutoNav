#include "DroneController.h"

double angleFromTo(double angle,double min,double max)
{
  while(angle<min)
    angle+=360;
  while(angle>max)
    angle-=360;
  return angle;
}

Controller::Controller()
{
  ROS_INFO("PID parameters for roll/pitch are: %lf,%lf,%lf",rp.Kp,rp.Kd,rp.Ki);
  
  velocity_channel = nh.resolveName("/cmd_vel");
  log_control_commands_channel = nh.resolveName("/log_control_commands");

  cmd_pub = nh.advertise<geometry_msgs::Twist>(velocity_channel,1);
  log_control_commands = nh.advertise<AutoNav::control_commands>(log_control_commands_channel,1);

  lastTimeStamp = 0;
  i_term =0;
  reached = false;
}

void Controller::i_increase(double new_err,double cap)
{
  if(new_err < 0 && i_term>0)
    i_term = std::max(0.0,i_term + 2.5*new_err);
  else if(new_err>0 && i_term<0)
    i_term = std::min(0.0,i_term + 2.5*new_err);
  else
    i_term+=new_err;

  i_term = std::max(-cap,std::min(i_term,cap));
}

void Controller::setGoal(Position newGoal)
{
  ROS_INFO("Accepting new goal of (%lf,%lf,%lf,%lf)",newGoal.x,newGoal.y,newGoal.z,newGoal.yaw);
  goal = newGoal;

  reached = false;
}

void Controller::sendControl(geometry_msgs::Twist cmd)
{
  cmd_pub.publish(cmd);
}

geometry_msgs::Twist Controller::calcControl(Vector4f error,Vector4f d_error,double cur_yaw)
{
  Vector4f p_term = error;
  Vector4f d_term = d_error;

  double yawRad = cur_yaw * 3.141592 / 180;

  AutoNav::control_commands message;

  message.error_x = error(0);
  message.error_y = error(1);
  message.error_z = error(2);
  message.error_yaw = error(3);

  message.d_error_x = d_error(0);
  message.d_error_y = d_error(1);
  message.d_error_z = d_error(2);
  message.d_error_yaw = d_error(3);

  p_term(0) = error(0)*cos(yawRad) + error(1)*sin(yawRad);
  p_term(1) = -error(0)*sin(yawRad) + error(1)*cos(yawRad);

  d_term(0) = d_error(0)*cos(yawRad) + d_error(1)*sin(yawRad);
  d_term(1) = -d_error(0)*sin(yawRad) + d_error(1)*cos(yawRad);

  message.proj_error_x = p_term(0);
  message.proj_error_y = p_term(1);

  double sec = getMS()/1000 - lastTimeStamp;
  lastTimeStamp = getMS()/1000;
  i_increase(error(2)*sec,0.2f/gaz.Ki);

  if(last_error*error(2)<0)
    i_term = 0;

  last_error = error(2);

  double gain_pX = p_term(0)*rp.Kp;
  double gain_pY  = p_term(1)*rp.Kp;
  double gain_pZ = p_term(2)*gaz.Kp;
  double gain_pYaw = p_term(3)*yaw.Kp;

  double gain_dX = d_term(0)*rp.Kd;
  double gain_dY= d_term(1)*rp.Kd;
  double gain_dZ = d_term(2)*gaz.Kd;
  double gain_dYaw = d_term(3)*yaw.Kd;
  
  double gain_iZ = i_term*gaz.Ki;

  geometry_msgs::Twist command;

  command.linear.x = std::max(-max_rp,std::min(max_rp,(double)(gain_pX+gain_dX)));
  command.linear.y = std::max(-max_rp,std::min(max_rp,(double)(gain_pY+gain_dY)));
  command.linear.z = std::max(min_gaz,std::min(max_gaz/rise_fac,(double)(gain_pZ + gain_dZ)));
  command.linear.z*= command.linear.z>0?rise_fac:1;
  command.angular.z = std::max(-max_yaw,std::min(max_yaw,(double)(gain_pYaw + gain_dYaw)));

  message.pterm_x = gain_pX;
  message.pterm_y = gain_pY;
  message.pterm_z = gain_pZ;
  message.pterm_yaw = gain_pYaw;

  message.dterm_x = gain_dX;
  message.dterm_y = gain_dY;
  message.dterm_z = gain_dZ;
  message.dterm_yaw = gain_dYaw;

  message.iterm_z = gain_iZ;

  message.vel_x = command.linear.x;
  message.vel_y = command.linear.y;
  message.vel_z = command.linear.z;
  message.vel_yaw = command.angular.z;

  log_control_commands.publish(message);

  return command;
}

void Controller::clearGoal()
{
  geometry_msgs::Twist hover;
  hover.linear.x = hover.linear.y = hover.linear.z = hover.angular.x = hover.angular.y = hover.angular.z = 0.0;
  reached = false;
  sendControl(hover);
}

bool Controller::update(const AutoNav::filter_stateConstPtr state)
{
  Vector4f error,d_error;

  error(0) = goal.x - state->x;
  error(1) = goal.y - state->y;
  error(2) = goal.z - state->z;
  error(3) = goal.yaw - state->yaw;

  d_error(0) = -state->dx;
  d_error(1) = -state->dy;
  d_error(2) = -state->dz;
  d_error(3) = -state->dyaw;

  sendControl(calcControl(error,d_error,state->yaw));

  if(reached && (getMS()-initReachClock) > stayTimeMS)
    {
      //ROS_INFO("Checkpoint done!");
      return true;
    }

  double squaredDistError = error(0)*error(0) + error(1)*error(1);

  if(!reached && squaredDistError<(initStayDist*initStayDist) && std::abs(error(3))<5)
    {
      reached = true;
      initReachClock = getMS();
      ROS_INFO("Reached buffer zone for target, waiting for stable hover!");
    }

  if(reached && (squaredDistError>(stayWithinDist*stayWithinDist) || std::abs(error(3))>5))
    {
      reached = false;
      ROS_INFO("Revoking reached status...");
    }

  return false;
}
