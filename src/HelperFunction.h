#pragma once

#ifndef __HELPERFUNCTIONS_H
#define __HELPERFUNCTIONS_H
 
 
#include <stdlib.h>
#include <ros/ros.h>

extern unsigned int ros_header_timestamp_base;
// gets a relative ms-time from a ROS time.
// can only be used to compute time differences, as it has no absolute reference.
inline static int getMS(ros::Time stamp = ros::Time::now())
{
  if(ros_header_timestamp_base == 0)
    {
      ros_header_timestamp_base = stamp.sec;
      ROS_INFO("set ts base to %d",ros_header_timestamp_base);
    }
  int mss = (stamp.sec - ros_header_timestamp_base) * 1000 + stamp.nsec/1000000;

  //ROS_INFO("getMS for %d.%d = %d",stamp.sec,stamp.nsec,mss);
  //  if(mss < 0)
    //    ROS_INFO("ERROR: negative timestamp for %d.%d=%d",stamp.sec,stamp.nsec,mss);
  return mss;
}

#endif

