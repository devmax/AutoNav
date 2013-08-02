#include "Teleop.h"
 
Keypress::Keypress(QWidget *parent):QWidget(parent)
{
  rosthread = NULL;

  myLabel = new QLabel("Label");
  mainLayout = new QVBoxLayout;
  mainLayout->addWidget(myLabel);
  setLayout(mainLayout);
  myLabel->setText("Welcome to keyboard control!\n"
		   "---------------------------\n"
		   "Takeoff/Land: q/a \n"
		   "Roll/Pitch: i,k / j,l\n"
		   "Throttle/Yaw: w,s / u,o\n"
		   " f to flat trim\n"
		   " t to toggle emergency state\n"
		   " r to reset filter\n"
		   " x/y/z to scale the PTAM thread\n"
		   " h to hover with the AutoPilot\n"
		   " m to revert to manual control\n"
		   " 1,2 / 3,4 to change linear/angular\n"
		   " speed limits by 10% \n"
		   " ESC to quit!\n");

  for(int i=0;i<8;i++)
    isPressed[i] = false;
}

Keypress::~Keypress()
{

}
void Keypress::keyPressEvent(QKeyEvent *key)
{
  int idx = mapKey(key->key());

  if(idx >= 0)
    {
      bool changed = !isPressed[idx];

      isPressed[idx] = true;

      if(changed)
	{
	  rosthread->publishCommand(calcKBControl());
	}
    }
  else if(key->key() == 81) //q
    {
      ROS_INFO("Takeoff!");
      rosthread->sendTakeOff();
    }
  else if(key->key() == 65) //a
    {
      rosthread->sendLand();
      ROS_INFO("Land!");
    }
  else if(key->key() == 70) //f
    {
      rosthread->sendFlatTrim();
      //ROS_INFO("Flat Trim!");
    }
  else if(key->key() == 84) //t
    {
      rosthread->sendToggleState();
      ROS_INFO("Reset State!");
    }
  else if(key->key() == 88) //x
    {
      rosthread->publishCom("Scale X");
      //ROS_INFO("Scaling on the x-axis");
    }
  else if(key->key() == 89) //y
    {
      rosthread->publishCom("Scale Y");
      //ROS_INFO("Scaling on the y-axis");
    }
  else if(key->key() == 90) //z
    {
      rosthread->publishCom("Scale Z");
      //ROS_INFO("Scaling on the z-axis");
    }
  else if(key->key() == 82)
    {
      rosthread->publishCom("Reset");
    }
  else if(key->key() == 49) //1
    {
      rosthread->speed *= 1.1;
      rosthread->speed = std::max(0.0,std::min(rosthread->speed,1.0));
      ROS_INFO("Speed is %lf",rosthread->speed);
    }
  else if(key->key() == 50) //2
    {
      rosthread->speed *= 0.9;
      rosthread->speed = std::max(0.0,std::min(rosthread->speed,1.0));
      ROS_INFO("Speed is %lf",rosthread->speed);
    }
  else if(key->key() == 51) //3
    {
      rosthread->turn *= 1.1;
      rosthread->turn = std::max(0.0,std::min(rosthread->turn,1.0));
      ROS_INFO("Turn is %lf",rosthread->turn);
    }
  else if(key->key() == 52) //4
    {
      rosthread->turn *= 0.9;
      rosthread->turn = std::max(0.0,std::min(rosthread->turn,1.0));
      ROS_INFO("Turn is %lf",rosthread->turn);
    }
  else if(key->key() == 72) //h
    {
      rosthread->autohover();
    }
  else if(key->key() == 77) //m
    {
      rosthread->revertmanual();
    }
  else if(key->key() == 71) //g
    {
      rosthread->housefigure();
    }
  else if(key->key() == 16777216) //ESC
    {
      rosthread->keepRunning = false;
      ROS_INFO("Shutting down window!");
    }
}

void Keypress::keyReleaseEvent(QKeyEvent *key)
{

  int idx = mapKey(key->key());

  if(idx >= 0)
    {
      bool changed = false;

      if(!key->isAutoRepeat())
	{
	  changed = isPressed[idx];
	  isPressed[idx] = false;
	}

      if(changed)
	rosthread->publishCommand(calcKBControl());
    }
}

int Keypress::mapKey(int k)
{
  switch(k)
    {
    case 74: // j
      return 0;
    case 76: // l  
      return 1;
    case 73: // i
      return 2;
    case 75: // k
      return 3;
    case 85: // u
      return 4;
    case 79: // o
      return 5;
    case 87: // w
      return 6;
    case 83: // s
      return 7;
    }
  return -1;
}

geometry_msgs::Twist Keypress::calcKBControl()
{
  geometry_msgs::Twist cmd;

  if(isPressed[0]) //j
    cmd.linear.y = rosthread->speed;
  if(isPressed[1]) //l
    cmd.linear.y = -rosthread->speed;
  if(isPressed[2]) //i
    cmd.linear.x = rosthread->speed;
  if(isPressed[3]) //k
    cmd.linear.x = -rosthread->speed;
  if(isPressed[4]) //u
    cmd.angular.z = rosthread->turn;
  if(isPressed[5]) //o
    cmd.angular.z = -rosthread->turn;
  if(isPressed[6]) //w
    cmd.linear.z = rosthread->speed;
  if(isPressed[7]) //s
    cmd.linear.z = -rosthread->speed;

  return cmd;
}
