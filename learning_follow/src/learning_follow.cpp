#include <gaml-libsvm.hpp>
#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
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
#include "std_msgs/String.h"
#include "synchronisateur/getThetas.h"
#include "synchronisateur/getR.h"
#include "synchronisateur/moveThetas.h" 
#include "Etats.hpp"
#include "learning_follow.hpp"
#include "generationRandom.hpp"

ros::Publisher movebase;
ros::Subscriber sub;
ros::ServiceClient client_rmin;
ros::ServiceClient client_getThetas;
ros::ServiceClient client_moveThetas;
bool isBegin;

#define NB_TESTS_ALEATOIRES 2
#define NB_TESTS_PAR_PHASE 2
#define R_min 0.5   //rayon min pour mouvement de base aleatoire
#define R_max 1.5   //rayon max  pour mouvement de base aleatoire
#define AngleTest 30  //angle qui definit la zone de test en degrés



void moveArm(const Thetas& theta){ //On appelle le client MoveThetas
  synchronisateur::moveThetas srv;
  srv.request.theta1 = theta[0];
  srv.request.theta2 = theta[1];
  srv.request.theta3 = theta[2];
  srv.request.theta4 = theta[3];
  if (client_moveThetas.call(srv)){
  }
  else {
    std::cout<<"Move Fail"<<std::endl;
  }
}


Rayon vecteur_kinnect_objet(){ //On appelle le client getR
  synchronisateur::getR srv;
  Rayon r_courant;
  if (client_rmin.call(srv)) {
    r_courant = {
      srv.response.x,
      srv.response.y,
      srv.response.z
    };
  }
  else{
    std::cout<<"getR Fail"<<std::endl;
  }
  return r_courant;
}




Thetas oscillations(Rayon r,Thetas& theta, Rayon rmin,Thetas& dthetamin){

  Rayon rprim;
  Thetas dthetaprim;
  Rayon newrmin =  rmin;
  Thetas newdthetamin = dthetamin;
  Thetas epsilon; 

  for(int i =0;i< NB_TESTS_ALEATOIRES;i++){

    std::cout << "---------------------" << std::endl;
    std::cout << "Amelioration n°" << i << std::endl;

    epsilon = creationEpsilonAleatoire();
    dthetaprim = dthetamin+epsilon;
    moveArm(dthetaprim+theta);
    rprim  = vecteur_kinnect_objet();

    std::cout<< "Distance :  " << norme(rprim) << std::endl;

    if(norme(rprim)<norme(newrmin)){
      newrmin = rprim;
      newdthetamin = dthetaprim; 
    }

  }
  return newdthetamin;
}





void apprentissageAleatoire(){

  BaseEtats baseEtats;
  Thetas thetarandom;
  Thetas dtheta;
  Thetas dthetamin;
  Rayon rmin;
  Rayon r;
  Entree ent;
  std::list<gaml::libsvm::Predictor<Entree,double>> predictors;
  std::array<std::string,5> filenames = {{std::string("theta1.pred"),"theta2.pred","theta3.pred","theta4.pred","theta5.pred"}};

 std::list<gaml::libsvm::Predictor<Entree,double>> null_predictors;
  gaml::libsvm::Predictor<Entree,double> null_predictor(nb_nodes_of, fill_nodes);
  for(int i = 0; i < 5; i++) null_predictors.push_back(null_predictor);
  fonction g(output_of_array,null_predictors.begin(), null_predictors.end());


  for (int j=0;j< NB_TESTS_PAR_PHASE;j++){

    std::cout<<"Nouvelle boucle mise a jour de f"<<std::endl;
    std::cout<<"____________________"<< std::endl;
    std::cout<<"Nouvel essai random n°"<<j<<std::endl;

    r = {0,0,0};

    while(r[0] == 0 && r[1] == 0 && r[2] == 0){
      thetarandom = creationThetaRandom();
      moveArm(thetarandom);
      r = vecteur_kinnect_objet();
    }

    if(isBegin){
      dtheta = {.0,.0,.0,.0,0};
    }else{
      ent = {r[0],r[1],r[2],thetarandom[0],thetarandom[1],thetarandom[2],thetarandom[3],thetarandom[4]};
      dtheta = { 
	g(ent).theta1,
	g(ent).theta2,
	g(ent).theta3,
	g(ent).theta4,
	g(ent).theta5
      };
    }
 
    moveArm(thetarandom+dtheta);
    rmin = vecteur_kinnect_objet();
 
    std::cout<<"Thetas :"<<thetarandom<<" | Rayon :"<<r<< " | Distance :"<<norme(r)<<"  | dthetas :"<<dtheta<<std::endl;
 
    dthetamin = oscillations(r,thetarandom,rmin,dtheta);
    Etat etat (r,thetarandom,dthetamin);
    baseEtats.push_back(etat);
    std::cout<<"Amelioration dthetas :  "<<dthetamin<<std::endl;

  }

  std::cout<<"mise a jour de f"<<std::endl;
  g = calcul_f(baseEtats);
  isBegin = false;

}



//MAIN
int main(int argc, char **argv) {
  
  Initialisation(); //Initialisation GAML
  ros::init(argc, argv, "learning_follow");
  ros::NodeHandle n;
  movebase = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  client_rmin = n.serviceClient<synchronisateur::getR>("getR");
  client_getThetas = n.serviceClient<synchronisateur::getThetas>("getThetas");
  client_moveThetas = n.serviceClient<synchronisateur::moveThetas>("moveThetas");
  isBegin = true;
  sleep(1);



  while (ros::ok())
    { 
      apprentissageAleatoire();
    }
  return 0;

}
