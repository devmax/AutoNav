#include "Circle.h"

Circle::Circle()
{
  vel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  log_control = n.advertise<AutoNav::circle_control>("/log_circleControl",1);
  state = n.subscribe("/ardrone/predictedPose",2,&Circle::stateCB,this);
}

void Circle::dynConfCB(AutoNav::CircleParamsConfig &config,uint32_t level)
{
  radius = config.radius;
  angRes = config.angRes;
  latVel = config.latVel;
  angVel = config.angVel * PI/180;

  angVel = latVel/radius;
  ROS_INFO("Target latVel: %lf and radius: %lf",latVel,radius);
  ROS_INFO("Angular speed changed to %lf degrees/second",angVel*180/PI);

  atr.Kp = config.Kat_p;
  atr.Kd = config.Kat_d;
  atr.Ki = config.Kat_i;

  ctr.Kp = config.Kct_p;
  ctr.Kp = config.Kct_d;
  ctr.Kp = config.Kct_i;

  angular.Kp = config.angular_p;
  angular.Kd = config.angular_d;
  angular.Ki = config.angular_i;
}

void Circle::stateCB(const AutoNav::filter_stateConstPtr state)
{

  AutoNav::circle_control log_circle;

  Eigen::Vector2f error;
  Eigen::Vector3f d_error;
  Eigen::Vector2f proj_error,proj_vel;
  Eigen::Vector2f goal;

  double yawRad = state->yaw * PI / 180;

  goal(0) = radius*(1 - cos(yawRad));
  goal(1) = -radius*sin(yawRad);

  error(0) = goal(0) - state->x;
  error(1) = goal(1) - state->y;

  proj_error(0) = error(0)*cos(yawRad) + error(1)*sin(yawRad);
  proj_error(1) = -error(0)*sin(yawRad) + error(1)*cos(yawRad);

  proj_vel(0) = (state->dx)*cos(yawRad) + (state->dy)*sin(yawRad);
  proj_vel(1) = -(state->dx)*sin(yawRad) + (state->dy)*cos(yawRad);

  d_error(0) = - proj_vel(0);
  d_error(1) = (-latVel) - proj_vel(1);
  d_error(2) = angVel - ((state->dyaw)*PI/180);

  double CTgainP = proj_error(0)*ctr.Kp;
  double CTgainD = d_error(0)*ctr.Kd;

  double ATgainP = d_error(1)*atr.Kp;
  double ATgainI = 0;

  double ANGgainP = d_error(2)*angular.Kp;
  double ANGgainD = 0;

  geometry_msgs::Twist cmd;

  cmd.linear.x = CTgainP + CTgainD;
  cmd.linear.y = ATgainP + ATgainI;
  cmd.angular.z = ANGgainP + ANGgainD;

  //START LOGGING

  log_circle.yaw = state->yaw;

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
  log_circle.VerrA = d_error(2);

  log_circle.CTgainP = CTgainP;
  log_circle.CTgainD = CTgainD;

  log_circle.ATgainP = ATgainP;
  log_circle.ATgainI = ATgainI;

  log_circle.ANGgainP = ANGgainP;
  log_circle.ANGgainD = ANGgainD;

  //END LOGGING

  ROS_INFO("Velocity commands are: %lf,%lf,%lf",cmd.linear.x,cmd.linear.y,cmd.angular.z);
  vel.publish(cmd);
}
