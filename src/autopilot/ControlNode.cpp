#include "ControlNode.h"

ControlNode::ControlNode()
{
  goalSet = goalReached = false;

  state_channel = n.resolveName("/ardrone/predictedPose");
  command_channel = n.resolveName("/AutoNav/commands");

  state_sub = n.subscribe(state_channel,2,&ControlNode::stateCB,this);
  command_sub = n.subscribe(command_channel,5,&ControlNode::commandCB,this);

  controller = Controller();
}

void ControlNode::commandCB(const std_msgs::StringConstPtr str)
{
  current = str->data;
  if(str->data == "hover")
    {
      ROS_INFO("Starting Auto-pilot hover now!");
      goalSet = false;
    }
  else if(str->data == "manual")
    {
      ROS_INFO("Reverting to manual control now!");
      controller.clearGoal();
      goalSet = false;
      goalReached = true;
    }
}

Position ControlNode::stateToPosition(const AutoNav::filter_stateConstPtr state)
{
  Position target;
  target.x = state->x;
  target.y = state->y;
  target.z = state->z;
  target.yaw = state->yaw;

  return target;
}

void ControlNode::stateCB(const AutoNav::filter_stateConstPtr state)
{
  if(current == "hover")
    {
      if(!goalSet)
	beginHover(stateToPosition(state));
      goalReached = controller.update(state);
    }
  
}

void ControlNode::dynConfCB(AutoNav::AutopilotParamsConfig &config,uint32_t level)
{
  controller.rp.Ki = config.Ki_rp;
  controller.rp.Kd = config.Kd_rp;
  controller.rp.Kp = config.Kp_rp;

  controller.yaw.Ki = config.Ki_yaw;
  controller.yaw.Kd = config.Kd_yaw;
  controller.yaw.Kp = config.Kp_yaw;

  controller.gaz.Ki = config.Ki_gaz;
  controller.gaz.Kd = config.Kd_gaz;
  controller.gaz.Kp = config.Kp_gaz;

  controller.min_yaw = config.min_yaw;
  controller.max_yaw = config.max_yaw;
  controller.min_rp = config.min_rp;
  controller.max_rp = config.max_rp;
  controller.min_gaz = config.min_gaz;
  controller.max_gaz = config.max_gaz_rise;

  ROS_INFO("After dynConfg, Kp,Ki,Kd are:%lf,%lf,%lf",controller.rp.Kp,controller.rp.Ki,controller.rp.Kd);
}

void ControlNode::Loop()
{
  ros::Rate r(25);

  while(n.ok())
    {
      ros::spin();
      r.sleep();
    }
}

void ControlNode::beginHover(Position goal)
{
  controller.setGoal(goal);
  goalSet = true;
  ROS_INFO("goal set as (%lf,%lf,%lf,%lf)",goal.x,goal.y,goal.z,goal.yaw);
}
