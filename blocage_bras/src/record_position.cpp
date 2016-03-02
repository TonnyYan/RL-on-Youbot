//Programme qui enregistre les positions angulaires du bras
//afin de construire une banque de points valides, qui ne met pas en
//danger le robot et n'est pas trop dense

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

ros::Publisher pub;
ros::Subscriber sub;

void callback(const sensor_msgs::JointState& msg)
{//Des nouvelles coordonnées ont été publiées

  //1) On parcourt la banque de points pour que pas trop dense
  
  //Si Ok on l'enregistre
  ROS_INFO_STREAM("Coordonnée enregistrée");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "record_position");
  ros::NodeHandle n;
  pub = n.advertise<brics_actuator::JointPositions>("out", 1);
  sub = n.subscribe("in",1,callback);
  ros::Rate loop_rate(10);
  sleep(1);
  while (ros::ok())
    { ros::spinOnce();
      r.sleep();
    }
  return 0;
}
