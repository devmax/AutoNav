#include "Circle.h"

unsigned int ros_header_timestamp_base = 0;

int main(int argc,char** argv)
{
  ros::init(argc,argv,"drone_circle");

  ROS_INFO("Starting circle node now!");

  Circle circle;

  dynamic_reconfigure::Server<AutoNav::CircleParamsConfig> srv;
  dynamic_reconfigure::Server<AutoNav::CircleParamsConfig>::CallbackType f;
  f = boost::bind(&Circle::dynConfCB,&circle,_1,_2);
  srv.setCallback(f);

  ros::spin();
}
