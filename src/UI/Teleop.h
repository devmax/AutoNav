#ifndef _TELEOP_H_
#define _TELEOP_H_

#include <stdio.h>
#include <QtGui>
#include <QWidget>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include "Rosthread.h"

class Rosthread;

class Keypress : public QWidget
{
  //Q_OBJECT
 private:
  QLabel* myLabel;
  QVBoxLayout *mainLayout;

 public:  
  Keypress(QWidget *parent = 0);
  ~Keypress();

  Rosthread* rosthread;

  void keyPressEvent(QKeyEvent *key); 
  void keyReleaseEvent(QKeyEvent *key);

  int mapKey(int k);

  bool isPressed[8];

  geometry_msgs::Twist calcKBControl();
};  
 
#endif //Teleop.h
