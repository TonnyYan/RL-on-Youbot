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
#include <iterator>
#include <algorithm>
#include <array>
#include <functional>
#include <cmath>

#define  MIN_DIST_THRESHOLD 1

ros::Publisher pub;
ros::Subscriber sub;

std::string adresseTexte ="/usr/users/promo2017/germain_hug/catkin_ws/src/blocage_bras/donnees_points.txt";

typedef std::array<double, 4> Joints;
typedef std::vector<Joints>   JointsSet;

std::ostream& operator<<(std::ostream& os, const Joints& coord_recues) {
  os << coord_recues[0] << ' ' << coord_recues[1] << ' ' <<  coord_recues[2] << ' ' << coord_recues[3];
  return os;
}

std::istream& operator>>(std::istream& is, Joints& coord_recues) {
  is >> coord_recues[0] >> coord_recues[1] >> coord_recues[2] >> coord_recues[3];
  return is;
}

inline double sqr(double x) {return  x*x;}

double distance2(const Joints& coord1, const Joints coord2) {
  return sqr(coord1[0]-coord2[0])+sqr(coord1[1]-coord2[1])+sqr(coord1[2]-coord2[2])+sqr(coord1[3]-coord2[3]);
}
double string_to_double( const std::string& s )
{
  std::istringstream i(s);
  double x;
  if (!(i >> x))
    return 0;
  return x;
}

void callback(JointsSet& collection, const brics_actuator::JointPositionsConstPtr& msg) {
//Des nouvelles coordonnées ont été publiées
  Joints coord_recues = {
    msg->positions[0].value, 
    msg->positions[1].value, 
    msg->positions[2].value, 
    msg->positions[3].value
  };

  auto pt_plusproche = std::min_element(collection.begin(), collection.end(),
					boost::bind(distance2,std::ref(coord_recues),_1));

  Joints distances = {
    std::abs(coord_recues[0]-*pt_plusproche[0]),
    std::abs(coord_recues[1]-*pt_plusproche[1]),
    std::abs(coord_recues[2]-*pt_plusproche[2]),
    std::abs(coord_recues[3]-*pt_plusproche[3])
  };
  bool cond0(distances[0]< MIN_DIST_THRESHOLD);
  bool cond1(distances[1]< MIN_DIST_THRESHOLD);
  bool cond2(distances[2]< MIN_DIST_THRESHOLD);
  bool cond3(distances[3]< MIN_DIST_THRESHOLD);
  
  if(cond0 && cond1 && cond2 && cond3) {
    std::cout << "Distances : "<< distances << std::endl;
    pub.publish(msg);
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
  pub = n.advertise<brics_actuator::JointPositions>("out", 1);
  sub = n.subscribe<brics_actuator::JointPositions>("in",1,boost::bind(callback,boost::ref(ref_points),_1));
  ros::Rate r(10);
  sleep(1);
  while (ros::ok())
    { ros::spinOnce();
      r.sleep();
    }
  return 0;
}
