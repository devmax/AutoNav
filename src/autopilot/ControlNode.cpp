#include "ControlNode.h"

ControlNode::ControlNode()
{
  goalSet = goalReached = false;

  state_channel = n.resolveName("/ardrone/predictedPose");
  command_channel = n.resolveName("/AutoNav/commands");

  state_sub = n.subscribe(state_channel,2,&ControlNode::stateCB,this);
  command_sub = n.subscribe(command_channel,5,&ControlNode::commandCB,this);

  //filling in coordinates of house!

  //  house.push_back(Position(0,-1,0,0)); //move right
  house.push_back(Position(0,-2,0,0)); //move right
  //  house.push_back(Position(-1,-2,0,0)); //move backward
  house.push_back(Position(-2,-2,0,0)); //move backward
  //  house.push_back(Position(-2,-1,0,0)); //move left
  house.push_back(Position(-2,0,0,0)); //move left
  //  house.push_back(Position(-1,0,0,0)); //move forward
  house.push_back(Position(0,0,0,0)); //move forward
  house.push_back(Position(-1,-1,0,0)); //move backward, right diagonally
  house.push_back(Position(-2,-2,0,0)); //move backward, right diagonally
  //  house.push_back(Position(-2,-1,0,0)); //move left
  house.push_back(Position(-2,0,0,0)); //move left
  house.push_back(Position(-1,-1,0,0)); //move forward, right diagonally
  house.push_back(Position(0,-2,0,0)); //move forward, right diagonally
  //  house.push_back(Position(0,-1,0,0)); //move left
  house.push_back(Position(0,0,0,0)); //move left

  seq = house.begin();

  origin.x = origin.y = origin.yaw =0.0;

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
  else if(str->data == "house")
    {
      ROS_INFO("Flying house figure!");
      seq = house.begin();
      goalSet = false;
      goalReached = false;
    }
  else if(str->data == "manual")
    {
      ROS_INFO("Reverting to manual control now!");
      controller.clearGoal();
      goalSet = false;
      goalReached = true;
    }
  else if(str->data.substr(0,4) == "move")
    {
      ROS_INFO_STREAM(str->data);
      goalSet = goalReached = false;
    }
  else if(str->data == "origin")
    {
      ROS_INFO("Flying to origin!");
      goalSet = false;
      goalReached = false;
    }
}

void ControlNode::updateHouse(Position cur)
{
  if(seq == house.begin())
    {
      reference.x = reference.y = reference.yaw = 0.0;
      reference.z = cur.z;
      //reference = cur;
    }
  //ROS_INFO("Adding next waypoint!");
  beginHover(reference + (*seq));
  goalReached = false;

  seq++;

  if(seq == house.end())
    {
      seq = house.begin();
      current = "hover";
    }
}
Position ControlNode::stateToPosition(const AutoNav::filter_stateConstPtr state)
{
  return Position(state->x,state->y,state->z,state->yaw);
}

void ControlNode::stateCB(const AutoNav::filter_stateConstPtr state)
{
  if(current == "hover")
    {
      if(!goalSet)
	beginHover(stateToPosition(state));
      goalReached = controller.update(state);
    }
  else if(current == "house")
    {
      if((!goalSet) || goalReached)
	{
	  ROS_INFO("Updating goals!");
	  updateHouse(stateToPosition(state));
	}
      else
	{
	  goalReached = controller.update(state);
	}
    }
  else if(current.substr(0,4) == "move")
    {
      if(!goalSet)
	{
	  Position movement;
	  sscanf(current.c_str(),"move %lf %lf %lf %lf",&movement.x,&movement.y,&movement.z,&movement.yaw);
	  beginHover(stateToPosition(state)+movement);
	}
      goalReached = controller.update(state);
    }
  else if(current == "origin")
    {
      if(!goalSet)
	{
	  origin.z = state->z;
	  beginHover(origin);
	}
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
  controller.min_gaz = config.max_gaz_drop;
  controller.max_gaz = config.max_gaz_rise;

  controller.rise_fac = config.rise_fac;

  controller.initStayDist = config.initStayDist;
  controller.stayWithinDist = config.stayWithinDist;
  controller.stayTimeMS = config.stayTimeMS;

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
  if(current == "house")
    goal.yaw = 0;

  goal.z = std::max(goal.z,0.4);
  //  goal.yaw = 0.0;
  //  goal.z = 1.0;
  controller.setGoal(goal);
  goalSet = true;
  //ROS_INFO("goal set as (%lf,%lf,%lf,%lf)",goal.x,goal.y,goal.z,goal.yaw);
}
