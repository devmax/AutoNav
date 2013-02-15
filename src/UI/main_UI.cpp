#include "Teleop.h"
#include "Rosthread.h"

int main(int argc, char **argv)
{  
  ros::init(argc,argv,"UI");

  Rosthread rthread;

  //std::cout<<"Rosthread object created! "<<&rthread;
 
  QApplication app(argc, argv);  
  
  Keypress box; 
  
  rthread.gui = &box;
  box.rosthread = &rthread;

  rthread.startSystem();
 
  box.show();  

  int ec = app.exec();

  ROS_INFO("Stopping ros thread now!");

  rthread.stopSystem();

  return ec;    
}  
