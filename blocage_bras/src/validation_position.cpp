//Programme qui analyse et filtre les coordonnées angulaires du bras
//afin de ne publier que celles qui ne le mettent pas en danger

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <array>
#include <functional>
#include <cmath>

#include "joints.hpp"

#define  MIN_DIST_THRESHOLD .4

ros::Publisher pub;
ros::Publisher feedback;
ros::Subscriber sub;

std::string adresseTexte ="/usr/users/promo2017/germain_hug/catkin_ws/src/blocage_bras/donnees_points.txt";


void callback(JointsSet& collection, const brics_actuator::JointPositionsConstPtr& msg) {
  //Des nouvelles coordonnées ont été publiées
  Joints coord_recues = {
    msg->positions[0].value, 
    msg->positions[1].value, 
    msg->positions[2].value, 
    msg->positions[3].value
  };

  bool pointvalide(true);
  double d = 0;
  for(auto& elt : collection){
    if( distance2(elt,coord_recues) < MIN_DIST_THRESHOLD*MIN_DIST_THRESHOLD){
      //SI on est trop près d'un point interdit, on se bloque
      pointvalide = false;
      d =  distance2(elt,coord_recues);
      break;
    }
  }

  if(pointvalide) {
    std::cout << "Point Valide" << std::endl;
    pub.publish(msg);
    std_msgs::StringPtr str(new std_msgs::String);
    str->data = "1";
    feedback.publish(str);
  }else{
    std::cout << "Point Non Valide "<< std::endl;
    std_msgs::StringPtr str(new std_msgs::String);
    str->data = "0";
    feedback.publish(str);
  }
}

JointsSet creationVecteur() {
  JointsSet  ref_points;
  auto out = std::back_inserter(ref_points);
  std::ifstream fichier;
  fichier.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);  
  try {
    fichier.open(adresseTexte.c_str());  
    try {
      while(true) {
	Joints joints;
	fichier >> joints;
	*(out++) = joints;
      }
    }
    catch (const std::exception& e) {
      // end of file...
    }
  }
  catch (const std::exception& e) {
    ROS_INFO_STREAM(e.what());
  }

  return ref_points;
}
int main(int argc, char **argv) {
  JointsSet ref_points = creationVecteur();
  ros::init(argc, argv, "record_position");
  ros::NodeHandle n;
  feedback = n.advertise<std_msgs::String>("feedback",1);
  pub = n.advertise<brics_actuator::JointPositions>("out", 1);
  sub = n.subscribe<brics_actuator::JointPositions>("in",1,boost::bind(callback,boost::ref(ref_points),_1));
  ros::Rate r(10);
  sleep(1);
  while (ros::ok()) { 
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
