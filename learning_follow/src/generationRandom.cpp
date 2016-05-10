#include <gaml-libsvm.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "Etats.hpp"
#include "learning_follow.hpp"
#include <string>
#include <stdio.h>
#include <iostream>

#define THETA1MAX 5.8401
#define THETA1MIN 0.01006
#define THETA2MAX 2.6179
#define THETA2MIN 0.0100
#define THETA3MAX -0.015
#define THETA3MIN -5.025
#define THETA4MAX 3.4292 
#define THETA4MIN 0.0221239
#define THETA5 0.111
#define EPS 0.1


Thetas creationThetaRandom(){
  Thetas thetarandom;
  thetarandom[0] = gaml::random::uniform(THETA1MIN,THETA1MAX);
  thetarandom[1] = gaml::random::uniform(THETA2MIN,THETA2MAX)*EPS;
  thetarandom[2] = gaml::random::uniform(THETA3MIN,THETA3MAX)*EPS-1;
  thetarandom[3] = gaml::random::uniform(THETA4MIN,THETA4MAX)*EPS;
  return thetarandom;
}


Thetas creationEpsilonAleatoire(){
  Thetas epsilonrandom;
  epsilonrandom[0] = gaml::random::uniform(-EPS,EPS);
  epsilonrandom[1] = gaml::random::uniform(-EPS,EPS);
  epsilonrandom[2] = gaml::random::uniform(-EPS,EPS);
  epsilonrandom[3] = gaml::random::uniform(-EPS,EPS);
  std::cout << "Thetas :" << epsilonrandom << std::endl;
  ros::Duration(0.5).sleep();
  return epsilonrandom;
}


void moveBaseRandom(ros::Publisher movebase){
  geometry_msgs::Twist twist;
  twist.linear.x = gaml::random::uniform(-EPS,EPS);
  twist.linear.y = gaml::random::uniform(-EPS,EPS);
  movebase.publish(twist);
  ros::Duration(0.5).sleep();
  twist.linear.x = 0;
  twist.linear.y = 0;
  movebase.publish(twist);
  ros::Duration(0.5).sleep();
}

