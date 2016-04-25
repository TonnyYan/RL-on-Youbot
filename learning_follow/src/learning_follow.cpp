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
#include <math.h>  

//TODO : 
//Résoudre pb Etat constructeur
//Remplir le main
//Créer un serveur qui gère les requetes

ros::Publisher movebase;
ros::Subscriber sub;
ros::ServiceClient client_rmin;
ros::ServiceClient client_getThetas;
ros::ServiceClient client_moveThetas;


#define NB_TESTS_ALEATOIRES 10
#define NB_TESTS_PAR_PHASE 10
#define R_min 0.5   //rayon min pour mouvement de base aleatoire
#define R_max 1.5   //rayon max  pour mouvement de base aleatoire
#define AngleTest 30  //angle qui definit la zone de test en degrés


typedef std::array<double, 3> Rayon; //vecteur à 3 dim r
typedef std::array<double, 5> Thetas;//vecteur à 5 dim Theta (seul les 4 premiers sont utilisés)


class Etat{
  Rayon r;
  Thetas theta;
  Thetas dthetamin;

public :

  Etat(const Rayon& r, const Thetas& theta, const Thetas& dthetamin)
  : r(r), theta(theta), dthetamin(dthetamin)  {
  }
  Etat()                       = default;
  Etat(const Etat&)            = default;
  Etat& operator=(const Etat&) = default;
};

typedef std::vector<Etat> BaseEtats;

Thetas  operator+(Thetas& theta1,Thetas& theta2) {
  Thetas out;
  out[0]=theta1[0]+theta2[0];
  out[1]=theta1[1]+theta2[1];
  out[2]=theta1[2]+theta2[2];
  out[3]=theta1[3]+theta2[3];
  out[4]=theta1[4]+theta2[4];
  return out;
}

std::ostream& operator<<(std::ostream& os, const Thetas& theta) {
  os << theta[0] << ' ' << theta[1] << ' ' <<  theta[2] << ' ' << theta[3]<< ' ' << theta[4];
  return os;
}
std::ostream& operator<<(std::ostream& os, const Rayon& r) {
  os << r[0] << ' ' << r[1] << ' ' <<  r[2];
  return os;
}

inline double sqr(double x) {return  x*x;}

double norme(Rayon r) {
  double norme = sqrt(sqr(r[0])+sqr(r[1])+sqr(r[2]));
  return norme;
}


Thetas creationThetaRandom(){
  Thetas thetarandom;
  //rand() returns a pseudo-random integral number in the range between 0 and RAND_MAX.
  //les valeurs des angles sont ramenes a des entiers entre 0 et ? pour avoir une precision de 0.01
  //par exemple joint 1 :  between 0.0100692 and 5.84014 ---> entre 1 et 584 -->(rand()% 584 +1)/100
  thetarandom[0]=(rand()% 584 +1.0)/1000;
  thetarandom[1]=(rand()% 261 +1.0)/1000 ;
  thetarandom[2]=(rand()% 501 +2.0)/1000*(-1);
  thetarandom[3]=(rand()% 340 +3.0)/1000;
  thetarandom[4]=0.111;

  return thetarandom;
}

//fonction qui bouge la base random en publiant sur le topic /out/base
void  moveBaseRandom(){
//on a un couple (r,theta) au hasard dans [rmin,rmax]*[0,angletest]
  double r =  ((double) rand() / (RAND_MAX))*(R_max-R_min)+R_min;
  double angle =  ((double) rand() / (RAND_MAX))*Angletest;

  // On transpose dans le repere (x,y)
  double x=r*cos(angle);
  double y = r*sin(angle);


  geometry_msgs::Twist twist;

  twist.linear.x = rand()/5-0.1;
  twist.linear.y = rand()/5-0.1;
  movebase.publish(twist);
  ros::Duration(0.5).sleep();
  twist.linear.x = 0;
  twist.linear.y = 0;
  movebase.publish(twist);
  ros::Duration(0.5).sleep();
}


//fonction qui bouge le bras d'un angle dtheats (a 4D) en publiant sur le topic /out qui est relié au topic /in de validation_position.cpp    

void moveArm(const Thetas& theta){
  //on cree un msg de type brics_actuator::JointPositions   ???? ou brics_actuator::JointPositionsConstPtr
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

//fonction qui renvoie un rayon r qui est le vecteur base_kinnect->objet
Rayon vecteur_kinnect_objet(){
  synchronisateur::getR srv;
  Rayon r_courant;
  if (client_rmin.call(srv)) {
    r_courant = {srv.response.x,srv.response.y,srv.response.z};
  }
  else{
    std::cout<<"getR Fail"<<std::endl;
  }
  return r_courant;

}

Thetas creationEpsilonAleatoire(){
  Thetas epsilonrandom;
 epsilonrandom[0]=(rand()% 21 -10.0)/100;
  epsilonrandom[1]=(rand()% 21 -10.0)/100;
  epsilonrandom[2]=(rand()% 21 -10.0)/100;
 epsilonrandom[3]=(rand()% 21 -10.0)/100;
 epsilonrandom[4]=0.111;
 std::cout<<"Thetas :"<<epsilonrandom<<std::endl;
ros::Duration(0.5).sleep();
  return epsilonrandom;
}


//fonction qui prend en argument une situation donnée (theta, r, dthemamin,rmin) et un entier n fixé  qui effectue des mouvement dtheta aleatoire autours de dthethamin initial pour essayer de trouver un nouveau mouvement dtheta plus interressant (i.e. tq r'<rmin)
//cette fonction renvoie le nouveau couple (r',dtheta') optimisé après n essais aléatoire
Thetas  mvtAleatoire(Rayon r,Thetas& theta, Rayon rmin,Thetas& dthetamin){
  Rayon rprim;
  Thetas dthetaprim;
  Rayon newrmin =  rmin;
  Thetas newdthetamin = dthetamin;
  Thetas epsilon; 
  for(int i =0;i< NB_TESTS_ALEATOIRES;i++){
    //modification de dtheta
    std::cout<<"Amelioration n°"<<i<<std::endl;
epsilon = creationEpsilonAleatoire();
    dthetaprim = dthetamin+epsilon;
    //on bouge de dthetaprim
    moveArm(dthetaprim+theta);
    //on regarde la nouvelle valeur de r
    rprim  = vecteur_kinnect_objet();
    //si rprim < rmin alors mise a jour de dthetamin et rmin
    if(norme(rprim)<norme(newrmin)){
      newrmin = rprim;
      newdthetamin = dthetaprim; 
    }
  }
  return newdthetamin;
}

//fonction qui prend un (theta,r) random et essaie d'ameliorer le dtheta, i.e. f(theta,r)
void apprentissageAleatoire(){
  BaseEtats baseEtats;
  Thetas thetarandom;
  Thetas dtheta;
  Thetas dthetamin;
  Rayon rmin;
  Rayon r;
  std::cout<<"Nouvelle boucle mise a jour de f"<<std::endl;
  for (int j=0;j< NB_TESTS_PAR_PHASE;j++){
    std::cout<<"Nouvel essai random n°"<<j<<std::endl;
    //on prend un (theta,r) random et on bouge le bras vers ce theta
    thetarandom = creationThetaRandom();
    moveArm(thetarandom);
    //   moveBaseRandom(); //-> Utiliser service
    r = vecteur_kinnect_objet();
    //on bouge le bras de dtheta que le programme a prévu i.e. f(thetarandom,r) et on regarde rmin qui est atteint avec le programme
    // dtheta = 0; //f=0
    dtheta = {.0,.0,.0,.0,0};
    moveArm(thetarandom+dtheta);
    rmin = vecteur_kinnect_objet();

    std::cout<<"Thetas :"<<thetarandom<<" | Rayon :"<<r<< " | Distance :"<<norme(r)<<"  | dthetas :"<<dtheta<<std::endl;
    //on bouge n fois autours de dtheta pour trouver un meilleur rmin et le dtheta qui lui est associé
    dthetamin = mvtAleatoire(r,thetarandom,rmin,dtheta);
    //creation d'un nouvel etat et ajout a la base de données
    Etat etat (r,thetarandom,dthetamin);
    baseEtats.push_back(etat);
    std::cout<<"Amelioration dthetas :  "<<dtheta<<std::endl;
  }
  //histoire du gaml avec mise a jour de f=learn(baseEtats)
  std::cout<<"mise a jour de f"<<std::endl;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "learning_follow");
  ros::NodeHandle n;
  movebase = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  client_rmin = n.serviceClient<synchronisateur::getR>("getR");
  client_getThetas = n.serviceClient<synchronisateur::getThetas>("getThetas");
  client_moveThetas = n.serviceClient<synchronisateur::moveThetas>("moveThetas");

  sleep(1);

  while (ros::ok())
    { 
      apprentissageAleatoire();
    }
  return 0;
}
