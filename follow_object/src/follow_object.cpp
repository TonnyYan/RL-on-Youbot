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

ros::Publisher movebase;
ros::Subscriber sub;
ros::ServiceClient client_rmin;
ros::ServiceClient client_getThetas;
ros::ServiceClient client_moveThetas;

fonction f;

#define R_max 1.5   //distance objet  max  pour mouvement




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




//On appelle le client getR
Rayon vecteur_kinnect_objet(){ 
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





//On appelle le client getThetas
Thetas vecteur_bras_objet(){ 
synchronisateur::geThetas srv;
Thetas thetas_courant;
if (client_getThetas.call(srv)) {
thetas_courant = {
srv.response.theta1,
  srv.response.theta2,
srv.response.thetas3,
srv.response.thetas4,
0.111
  };
}
 else{
std::cout<<"getThetas Fail"<<std::endl;
}
return thetas_courant;
}




bool chargement_f(){

std::array<std::string,5> filenames = {{std::string("theta1.pred"),"theta2.pred","theta3.pred","theta4.pred","theta5.pred"}};
std::list<gaml::libsvm::Predictor<Entree,double>> predictors;
name_iter= filenames.begin();

    for(unsigned int dim = 0; dim < 5; ++dim) {
      gaml::libsvm::Predictor<Entree,double> predictor(nb_nodes_of, fill_nodes);
      predictor.load_model(*(name_iter++));
      predictors.push_back(predictor);
    }

    gaml::multidim::Predictor<Point,
                              gaml::libsvm::Predictor<Entree,double>,
                              5> f(output_of_array, predictors.begin(), predictors.end());

 return true;
}


//boucle principale
void autoSuivi(){

Thetas theta;
Rayon r;
Entree ent;
bool pas_incident =true;
Thetas dtheta;
double amelioration_distance=0;
double distance_precedante=0;



while(pas_incident){

std::cout<<"____________________"<< std::endl;

//lecture entree
theta = vecteur_bras_objet();
r = vecteur_kinnect_objet();

//mouvArm adequate par f
ent = {r[0],r[1],r[2],theta[0],theta[1],theta[2],theta[3],theta[4]};
dtheta = {
f(ent).theta1,
  f(ent).theta2,
  f(ent).theta3,
  f(ent).theta4,
  f(ent).theta5
  };

moveArm(theta+dtheta);

//stabilisation de r, theta
sleep(0.5);

//affichage des valeurs de r, dtheta=f(r,theta),distance  pour suivre l'evolution
 amelioration_distance = distance_precedante-norme(r);
 distance_precedante = norme(r);
 std::cout<<"| Rayon :"<<r<< "    | Distance :"<<norme(r)<<"  |   dthetas :"<<dtheta<<" | Distance :"<<amelioration_distance<< std::endl;

}











//MAIN
int main(int argc, char **argv) {
  
//Initialisation GAML et chargement de f
Initialisation_params();
 if(!chargement_f()){
   std::cout<<"Erreur de chargement"<<std::endl;
   return 0;}



ros::init(argc, argv, "follow_object");
ros::NodeHandle n;

//Initialisation de la lecture de R,Thetas en in et cmd_vel moveThetas en out
movebase = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
client_rmin = n.serviceClient<synchronisateur::getR>("getR");
client_getThetas = n.serviceClient<synchronisateur::getThetas>("getThetas");
client_moveThetas = n.serviceClient<synchronisateur::moveThetas>("moveThetas");
sleep(1);



while (ros::ok())
  { 
autoSuivi();
}
return 0;

}
