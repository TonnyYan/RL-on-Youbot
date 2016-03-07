//Programme qui analyse et filtre les coordonnées angulaires du bras
//afin de ne publier que celles qui ne le mettent pas en danger

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <array>
#include <cmath>


ros::Publisher pub;
ros::Subscriber sub;

std::string adresseTexte ="//usr//users//promo2017//bordogna_rem//catkin_ws//src//blocage_bras//donnees_points.txt";

std::vector<std::array<double, 4>> ref_points;//Les points de référence

double distance(const std::array<double,4>coord1, const std::array<double,4> coord2) {
  double distance = sqrt(pow((coord1[0]-coord2[0]),2)+pow((coord1[1]-coord2[1]),2)+pow((coord1[2]-coord2[2]),2)+pow((coord1[3]-coord2[3]),2));
return distance;
 }

double string_to_double( const std::string& s )
{
  std::istringstream i(s);
  double x;
  if (!(i >> x))
    return 0;
  return x;
}


void callback(const brics_actuator::JointPositions& msg)
{//Des nouvelles coordonnées ont été publiées
  std::array<double,4> coord_recues = {msg.positions[0].value, msg.positions[1].value, msg.positions[2].value, msg.positions[3].value};
  double distance_min(1000);
  for(auto& elt : ref_points){
    if(distance(elt,coord_recues)<distance_min)
      distance_min = distance(elt,coord_recues);
  }
  std::cout << "Distance min :" << distance_min << std::endl;
  bool point_valide = (distance_min<0.3);
  if(point_valide) pub.publish(msg);
}


std::vector<std::array<double,4>> creationVecteur(){//Renvoie le vecteur de points
  std::string ligne;
  std:: string nb="";
  std::vector<std::array<double, 4>>  ref_points;
  try {
    std::ifstream fichier(adresseTexte.c_str(), std::ios::in);    
    //  fichier.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);//Test si erreur
    while(getline(fichier,ligne)){
      std::array<double, 4> uneLigne;
      for(int i =0; i<5; i++){nb=nb+ligne[i];}
      uneLigne[0]=string_to_double(nb);
      nb="";
      for(int i =6; i<10; i++){nb=nb+ligne[i];}
      uneLigne[1]=string_to_double(nb);
      nb="";
      for(int i =12; i<16; i++){nb=nb+ligne[i];}
      uneLigne[2]=string_to_double(nb);
      nb="";
      for(int i =17; i<21; i++){nb=nb+ligne[i];}
      uneLigne[3]=string_to_double(nb);
      nb="";
      ref_points.push_back(uneLigne);
    }
  }
  catch (const std::exception& e) {
    ROS_INFO_STREAM("ERREUR Ouverture Fichier");
  }
  return ref_points;
}

int main(int argc, char **argv) {
  ref_points = creationVecteur();
  ros::init(argc, argv, "record_position");
  ros::NodeHandle n;
  pub = n.advertise<brics_actuator::JointPositions>("out", 1);
  sub = n.subscribe("in",1,callback);
  ros::Rate r(10);
  sleep(1);
  while (ros::ok())
    { ros::spinOnce();
      r.sleep();
    }
  return 0;
}
