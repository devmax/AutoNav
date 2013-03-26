#include "Interact.h"

//Interact marker_interact;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server;

Interact::Interact()
{
  rosthread = NULL;
  reference.x = reference.y = reference.z = reference.yaw = 0;
  manual_command.x = manual_command.y = manual_command.z = manual_command.yaw = 0;

  int_marker.name = "Interactive Teleop";
  manual_marker.name = "Manual Teleop";
}
void Interact::start()
{
  marker_server.reset(new interactive_markers::InteractiveMarkerServer("drone_controls","",false) );

  menu_handler.insert("Takeoff",processFeedback);
  menu_handler.insert("Land",processFeedback);
  menu_handler.insert("Hover",processFeedback);
  menu_handler.insert("Manual",processFeedback);

  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Set");
  menu_handler.insert(sub_menu_handle,"Flat Trim",processFeedback);
  menu_handler.insert(sub_menu_handle,"Reset State",processFeedback);

  menu_handler_manual.insert("Takeoff",processFeedback);
  menu_handler_manual.insert("Land",processFeedback);
  menu_handler_manual.insert("Hover",processFeedback);
  menu_handler_manual.insert("Autonomous",processFeedback);

  interactive_markers::MenuHandler::EntryHandle sub_menu_handle_manual = menu_handler_manual.insert("Set");
  menu_handler_manual.insert(sub_menu_handle_manual,"Flat Trim",processFeedback);
  menu_handler_manual.insert(sub_menu_handle_manual,"Reset State",processFeedback);

  this->makeAutonomousMarker();
  //makeManualMarker();

  marker_server->applyChanges();

  ros::spin();

  marker_server.reset();
}

visualization_msgs::Marker Interact::makeBox(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.4;
  marker.scale.y = msg.scale * 0.4;
  marker.scale.z = msg.scale * 0.4;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->marker_name == marker_interact.int_marker.name)
      switch(feedback->event_type)
	{
	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
	  switch(feedback->menu_entry_id)
	    {
	    case 1:
	      marker_interact.rosthread->sendTakeOff();
	      break;
	    case 2:
	      marker_interact.rosthread->sendLand();
	      break;
	    case 3:
	      marker_interact.rosthread->autohover();
	      break;
	    case 4:
	      marker_server->clear();
	      marker_interact.makeManualMarker();
	      marker_server->applyChanges();
	      marker_interact.resetManual();
	      break;
	    case 6:
	      marker_interact.rosthread->sendFlatTrim();
	      break;
	    case 7:
	      marker_interact.rosthread->sendToggleState();
	      break;
	    }
	  break;
	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
	  marker_interact.setReference(feedback->pose);
	  break;
	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
	  marker_interact.generateCommandAuto(feedback->pose);
	  break;
	}
  else if(feedback->marker_name == marker_interact.manual_marker.name)
      switch(feedback->event_type)
	{
	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
	  switch(feedback->menu_entry_id)
	    {
	    case 1:
	      marker_interact.rosthread->sendTakeOff();
	      break;
	    case 2:
	      marker_interact.rosthread->sendLand();
	      break;
	    case 3: 
	      marker_interact.resetManual();
	      break;
	    case 4:
	      marker_server->clear();
	      marker_interact.makeAutonomousMarker();
	      marker_server->applyChanges();
	      marker_interact.rosthread->autohover();
	      break;
	    case 6:
	      marker_interact.rosthread->sendFlatTrim();
	      break;
	    case 7:
	      marker_interact.rosthread->sendToggleState();
	      break;
	    }
	  break;
	case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
	  marker_interact.generateCommandManual(feedback->pose);
	  ROS_INFO("Sending teleop velocities now..");
	  break;
	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
	  marker_interact.resetManual();
	  break;
	}
  
  marker_server->applyChanges();
}

Position Interact::poseToPosition(geometry_msgs::Pose pose)
{
  Position pos;
  pos.x = pose.position.x;
  pos.y = pose.position.y;
  pos.z = pose.position.z;

  double r,p,y;
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose,tf_pose);
  tf_pose.getBasis().getRPY(r,p,y);

  pos.yaw = y*180/PI;

  return pos;
}

void Interact::setReference(geometry_msgs::Pose pose)
{
  this->reference = this->poseToPosition(pose);
}

Position Interact::findDifference(geometry_msgs::Pose pose)
{
  return (this->poseToPosition(pose)-this->reference);
}

void Interact::resetManual()
{
  manual_command.x = manual_command.y = manual_command.z = manual_command.yaw = 0;

  geometry_msgs::Pose pose;
  
  marker_server->setPose(manual_marker.name,pose);
}

void Interact::generateCommandAuto(geometry_msgs::Pose pose)
{
  this->rosthread->publishCommand(this->findDifference(pose));
}

void Interact::generateCommandManual(geometry_msgs::Pose pose)
{
  Position diff = this->poseToPosition(pose);

  diff.x = diff.x>0?ceil(diff.x-0.25):floor(diff.x+0.25);
  diff.y = diff.y>0?ceil(diff.y-0.25):floor(diff.y+0.25);
  diff.z = diff.z>0?ceil(diff.z-0.25):floor(diff.z+0.25);

  this->manual_command.x = diff.x*0.25;
  this->manual_command.y = diff.y*0.25;
  this->manual_command.z = diff.z*0.25;
  this->manual_command.yaw = diff.yaw/60;

  //ROS_INFO("Generated (%lf,%lf,%lf,%lf)",manual_command.x,manual_command.y,manual_command.z,manual_command.yaw);
}

geometry_msgs::Twist Interact::calcControl()
{
  geometry_msgs::Twist cmd;

  cmd.linear.x = manual_command.x;
  cmd.linear.y = manual_command.y;
  cmd.linear.z = manual_command.z;
  cmd.angular.z = manual_command.yaw;

  return cmd;
}

void Interact::makeAutonomousMarker()
{
  int_marker.header.frame_id = "/ardrone_base_link";
  int_marker.pose.position.x = int_marker.pose.position.y = int_marker.pose.position.z = 0;
  int_marker.scale = 1;

  int_marker.name = "Interactive Teleop";
  int_marker.description = "AR.Drone";

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  control.name = "auto_roll/pitch/yaw";
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.always_visible = true;
  control.name = "auto_gaz";
  int_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.name = "auto_menu";

  visualization_msgs::Marker marker = makeBox(int_marker);
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);

  marker_server->insert(int_marker);
  marker_server->setCallback(int_marker.name,&processFeedback);
  menu_handler.apply(*marker_server,int_marker.name);
}

void Interact::makeManualMarker()
{
  manual_marker.header.frame_id = "/ardrone_base_link";
  manual_marker.pose.position.x = manual_marker.pose.position.y = manual_marker.pose.position.z = 0;
  manual_marker.scale = 1;

  manual_marker.name = "Manual Teleop";
  manual_marker.description = "AR.Drone(M)";

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.name = "manual_yaw";
  control.always_visible = true;
  manual_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.always_visible = true;
  control.name = "manual_gaz";
  manual_marker.controls.push_back(control);
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "manual_pitch";
  manual_marker.controls.push_back(control);
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "manual_roll";
  manual_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.name = "manual_menu";

  visualization_msgs::Marker marker = makeBox(manual_marker);
  control.markers.push_back(marker);
  manual_marker.controls.push_back(control);

  marker_server->insert(manual_marker);
  marker_server->setCallback(manual_marker.name,&processFeedback);
  menu_handler_manual.apply(*marker_server,manual_marker.name);
}
