
#include "Controller.h"

double angleFromTo(double angle, double min, double max)
{
  while(angle<min)
    angle+=360;
  while(angle>=max)
    angle-=360;
  return angle;
}
DroneController::DroneController()
{
  Kp_xy = 0.25;//5;
  Kd_xy = 0.16;//35;
  Ki_xy = 0.0;
  Kp_gaz = 0;// 0.6;
  Kd_gaz = 0;//0.1;
  Ki_gaz = 0;//0.01;
  Kp_yaw = 0;//0.05;
  Kd_yaw = 0.0;

  err_xy = 0.2;
  err_z = 0.3;
  err_yaw = 3;

  rise_fac = 2.5;

  init_state = (Vector4f()<<0.0,0.0,0.0,0.0).finished();
  inited= false;

  state_channel = nh.resolveName("/ardrone/predictedPose");
  velocity_channel = nh.resolveName("/cmd_vel");
  control_commands_channel = nh.resolveName("/log_control_commands",1);

  cmd_pub = nh.advertise<geometry_msgs::Twist>(velocity_channel,1);
  state_pub = nh.subscribe(state_channel,1,&DroneController::stateCB,this);
  pub_control_commands = nh.advertise<AutoNav::control_commands>(control_commands_channel,1);
}

void DroneController::calcControl(Vector4f error,Vector4f d_error,double yaw)
{
  Vector4f p_term = error;
  Vector4f d_term = d_error;

  double yawRad = yaw * 3.141592 / 180;
  //yawRad = angleFromTo(yawRad,-180,180);

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

  double gain_pX = p_term(0)*Kp_xy;
  double gain_pY  = p_term(1)*Kp_xy;
  double gain_pZ = p_term(2)*Kp_gaz;
  double gain_pYaw = p_term(3)*Kp_yaw;

  double gain_dX = d_term(0)*Kd_xy;
  double gain_dY= d_term(1)*Kd_xy;
  double gain_dZ = d_term(2)*Kd_gaz;
  double gain_dYaw = d_term(3)*Kd_yaw;

  geometry_msgs::Twist command;

  command.linear.x = std::max(-0.6,std::min(0.6,(double)(gain_pX+gain_dX)));
  command.linear.y = std::max(-0.6,std::min(0.6,(double)(gain_pY+gain_dY)));
  command.linear.z = gain_pZ + gain_dZ;
  command.angular.z = gain_pYaw + gain_dYaw;

  //ROS_INFO("X:Sum = %lf but publishing %lf",gain_pX+gain_dX,command.linear.x);
  //ROS_INFO("Y:Sum = %lf but publishing %lf",gain_pY+gain_dY,command.linear.y);

  if(((std::abs(command.linear.x) < 0.1) && (std::abs(command.linear.y) < 0.1) && (std::abs(command.linear.z) < 0.1) && (std::abs(command.angular.z) < 0.1)) || (std::abs(error(0)) < 0.3 && std::abs(error(1)) < 0.3))
    {
      ROS_INFO("Sending pseudo-hover with (%lf,%lf,%lf,%lf)!",std::abs(command.linear.x),std::abs(command.linear.y),std::abs(command.linear.z),std::abs(command.angular.z));
      command.linear.x = command.linear.y = command.linear.z = command.angular.z = 0.0;
      command.angular.x = command.angular.y = 1.0;
    }

  cmd_pub.publish(command);
  
  message.pterm_x = gain_pX;
  message.pterm_y = gain_pY;
  message.pterm_z = gain_pZ;
  message.pterm_yaw = gain_pYaw;

  message.dterm_x = gain_dX;
  message.dterm_y = gain_dY;
  message.dterm_z = gain_dZ;
  message.dterm_yaw = gain_dYaw;

  message.vel_x = command.linear.x;
  message.vel_y = command.linear.y;
  message.vel_z = command.linear.z;
  message.vel_yaw = command.angular.z;

  pub_control_commands.publish(message);
  
}

void DroneController::stateCB(const AutoNav::filter_stateConstPtr state)
{
  if(!inited)
    {
      init_state(0) = state->x;
      init_state(1) = state->y;
      init_state(2) = state->z;
      init_state(3) = state->yaw;

      ROS_INFO("Initial state is: %lf,%lf,%lf,%lf",init_state(0),init_state(1),init_state(2),init_state(3));
      inited = true;
    }
  Vector4f error, d_error;

  error(0) = init_state(0)-state->x;
  error(1) = init_state(1)-state->y;
  error(2) = init_state(2)-state->z;
  error(3) = init_state(3)-state->yaw;

  double yaw = state->yaw;

  d_error(0) = -state->dx;
  d_error(1) = -state->dy;
  d_error(2) = -state->dz;
  d_error(3) = -state->dyaw;

  calcControl(error,d_error,yaw);
}

void DroneController::Loop()
{
  ros::Rate r(30);

  while(nh.ok())
    {
      ros::spinOnce();

      r.sleep();
    }
}


