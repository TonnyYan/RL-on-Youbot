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
#include <string>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <array>
#include <functional>
#include <cmath>

#include "joints.hpp"


ros::Subscriber sub;

std::string adresseTexte ="/usr/users/promo2017/germain_hug/catkin_ws/src/blocage_bras/donnees_points.txt";

#define MIN_DIST_THRESHOLD .1



void callback(JointsSet& collection, const brics_actuator::JointPositionsConstPtr& msg) {//Des nouvelles coordonnées ont été publiées
  Joints coord_recues = {
    msg->positions[0].value, 
    msg->positions[1].value, 
    msg->positions[2].value, 
    msg->positions[3].value
  };

 
  auto min_iter = std::min_element(collection.begin(), collection.end(),
				   boost::bind(distance2,std::ref(coord_recues),_1));
  
  // Si Ok on l'enregistre
  if(distance2(coord_recues,*min_iter) > MIN_DIST_THRESHOLD*MIN_DIST_THRESHOLD) {
      std::ofstream fichier; 
      fichier.exceptions(std::ios::failbit | std::ios::badbit);
      try {
	fichier.open(adresseTexte.c_str(), std::ios_base::app);
	fichier << coord_recues << std::endl;
	fichier.close();
	collection.push_back(coord_recues);
	ROS_INFO_STREAM("Coordonnee enregistree");
	std::cout << coord_recues<<std::endl;
      }
      catch (const std::exception& e) {
	ROS_INFO_STREAM("ERREUR Ouverture Fichier");
      }
  }
}

JointsSet creationVecteur() {
  JointsSet  ref_points;
  auto out = std::back_inserter(ref_points);
  std::ifstream fichier;
  fichier.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);  
  try {
    fichier.open(adresseTexte.c_str(), std::ios_base::app);  
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
  sub = n.subscribe<brics_actuator::JointPositions>("in",1,boost::bind(callback, boost::ref(ref_points), _1));
  ros::Rate r(10);
  sleep(1);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
