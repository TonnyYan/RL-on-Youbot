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

//TODO : 
//Résoudre pb Etat constructeur
//Remplir le main
//Kinect vecteur r

ros::Publisher pub;
ros::Publisher movebase;
ros::Subscriber sub;

#define NB_TESTS_ALEATOIRES 10
#define NB_TESTS_PAR_PHASE 10

typedef std::array<double, 3> Rayon; //vecteur à 3 dim r
typedef std::array<double, 5> Thetas;//vecteur à 5 dim Theta (seul les 4 premiers sont utilisés)

class Etat{
  Rayon r;
  Thetas theta;
  Thetas dthetamin;
public :Etat(Rayon ,Thetas ,Thetas); //constructeur mais pas sur que ca marche
};

Etat::Etat (Rayon r1,Thetas theta1,Thetas dthetamin1){
  r=r1;
  theta=theta1;
  dthetamin=dthetamin1;
}
//pb !  typedef std::array<Etat,1> BaseEtats; 


 Thetas  operator+(Thetas& theta1,Thetas& theta2) {
  Thetas out;
  out[0]=theta1[0]+theta2[0];
  out[1]=theta1[1]+theta2[1];
  out[2]=theta1[2]+theta2[2];
  out[3]=theta1[3]+theta2[3];
  out[4]=theta1[4]+theta2[4];

  return out;
}

/* Theta&s operator=(Thetas& theta2) {
   Thetas out;
   out[0]=theta2[0];
   out[1]=theta2[1];
   out[2]=theta2[2];
   out[3]=theta2[3];
   out[4]=theta2[4];
   return out;
   }

   Theta&s operator=(int& tab[5] ) {
   Thetas out;
   theta1[0]=tab[0];
   theta1[1]=tab[1];
   theta1[2]=tab[2];
   theta1[3]=tab[3];
   theta1[4]=tab[4];

   return out;
   }*/



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
  thetarandom[0]=(rand()% 584 +1)/100;
  thetarandom[1]=(rand()% 261 +1)/100 ;
  thetarandom[2]=(rand()% 501 +2)/100*(-1);
  thetarandom[3]=(rand()% 340 +3)/100;
  thetarandom[4]=0.111;

  return thetarandom;
}

//fonction qui bouge la base random en publiant sur le topic /out/base
void  moveBaseRandom(){
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

void move(Thetas& theta){
  //on cree un msg de type brics_actuator::JointPositions   ???? ou brics_actuator::JointPositionsConstPtr
  brics_actuator::JointPositions msg; 
  for (int i = 0; i < 5; i++) {
    brics_actuator::JointValue joint;
    joint.timeStamp = ros::Time::now();
    joint.value = theta[i];
    joint.unit = boost::units::to_string(boost::units::si::radian);
    std::stringstream jointName;
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();
    //On empile le message value dans le message de positions
    msg.positions.push_back(joint);
  }
  pub.publish(msg);
}

//fonction qui renvoie un rayon r qui est le vecteur base_kinnect->objet
//elle lit sur un topic (partie Hugo)
Rayon vecteur_kinnect_objet(){
  Rayon r;
  //on récupère le point le plus proche avec pcl
  //a completer


  return r;

}




//fonction qui prend en argument une situation donnée (theta, r, dthemamin,rmin) et un entier n fixé  qui effectue des mouvement dtheta aleatoire autours de dthethamin initial pour essayer de trouver un nouveau mouvement dtheta plus interressant (i.e. tq r'<rmin)
//cette fonction renvoie le nouveau couple (r',dtheta') optimisé après n essais aléatoire
Thetas  mvtAleatoire(Rayon r,Thetas& theta, Rayon rmin,Thetas& dthetamin){
  Rayon rprim;
  Thetas dthetaprim;
  /*Rayon newrmin= rmin;
    Thetas newdthetamin= dthetamin;*/
  Rayon newrmin =  rmin;
  Thetas newdthetamin = dthetamin;
  Thetas epsilon= {0,0,0,0,0};
  for(int i =0;i< NB_TESTS_ALEATOIRES;i++){
    //modification de dtheta
    dthetaprim = dthetamin+epsilon;
    //on bouge de dthetaprim
    move(dthetaprim+theta);
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
    move(thetarandom);
    moveBaseRandom();
    r = vecteur_kinnect_objet();
    //on bouge le bras de dtheta que le programme a prévu i.e. f(thetarandom,r) et on regarde rmin qui est atteint avec le programme
    // dtheta = 0; //f=0
    dtheta = {0,0,0,0,0};
    move(thetarandom+dtheta);
    rmin = vecteur_kinnect_objet();

    std::cout<<"Thetas :"<<thetarandom<<" | Rayon :"<<r<< " | Distance :"<<norme(r)<<"dthetas :"<<dtheta<<std::endl;
    //on bouge n fois autours de dtheta pour trouver un meilleur rmin et le dtheta qui lui est associé
    dthetamin = mvtAleatoire(r,thetarandom,rmin,dtheta);
    //creation d'un nouvel etat et ajout a la base de données
     Etat etat (r,thetarandom,dthetamin);
    //pb ici!    BaseEtats baseEtats.pushback(etat);
    std::cout<<"Amelioration dthetas :  "<<dtheta<<std::endl;
  }
  //histoire du gaml avec mise a jour de f=learn(baseEtats)
  std::cout<<"mise a jour de f"<<std::endl;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "learning_follow");
  ros::NodeHandle n;
  pub = n.advertise<brics_actuator::JointPositions>("out/positions", 1);
  movebase = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Rate r(10);
  sleep(1);
  while (ros::ok())
    { ros::spinOnce();
      r.sleep();
    }
  return 0;
}
