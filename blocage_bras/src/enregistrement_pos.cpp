#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "curses.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <ctime>
using namespace std;

ros::Publisher brasPublisher;
double lesThetas[5];

// struct Thetas {
//   std::array<double,5> angles;

//   void in_bounds() {
//     if(angles[0]<.2) angles[0] = .2;
//   }
// };


// // voir surcharge d'operateurs


// std::ostream& operator<<(std::ostream& os, const Thetas& thetas) {
//   os << anges[0] << ' ' << ....;
// }

// file << thetas1 << ' ' << thetas2 << std::endl;

void affichageMessageIntro()
{

  string messageIntro("");
  messageIntro = "Bienvenue dans le manipulateur youbot \n \nCommandez le robot avec les touches suivantes : \n " ;
  messageIntro += "\n|  z : theta1 ++            |"; //
  messageIntro += "\n|  s : theta1 --            |";
  messageIntro += "\n|  e : theta2 ++            |";
  messageIntro += "\n|  d : theta2 --            |";
  messageIntro += "\n|  r : theta3 ++            |";
  messageIntro += "\n|  f : theta3 --            |";
  messageIntro += "\n|  t : theta4 ++            |";
  messageIntro += "\n|  g : theta4 --            |\n";
  cout << messageIntro;

}
void ecritureTexte(){
  ofstream fichier("//usr//users//promo2017//bordogna_rem//catkin_ws//src//blocage_bras//donnees_points.txt", ios::out | ios::app); 
  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );
  if(fichier)
    { 
      fichier << "Enregistrement du " << (now->tm_year + 1900) << '-'  << (now->tm_mon + 1) << '-'<<  now->tm_mday << endl;
      fichier.close();
    }
  else{
    cout << "Impossible d'ouvrir le fichier !" << endl;}
}


void Miseajour() {

  //Initialisation des valeurs
  std::vector<double> jointvalues(5);
  jointvalues[0] = lesThetas[0];//Rotation de la base
  jointvalues[1] = lesThetas[1];//Valeurs par défaut de calibration
  jointvalues[2] = lesThetas[2] ;
  jointvalues[3] = lesThetas[3];
  jointvalues[4] = 0.111;

  //Création de messages positions
  //Contient une pile de 5 messages de type JointValue
  brics_actuator::JointPositions msg;
  for (int i = 0; i < 5; i++) {
    //Initialisation du message value du joint no.i
    brics_actuator::JointValue joint;
    joint.timeStamp = ros::Time::now();
    joint.value = jointvalues[i];
    joint.unit = boost::units::to_string(boost::units::si::radian);
    std::stringstream jointName;
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();
    //On empile le message value dans le message de positions
    msg.positions.push_back(joint);
  }

  //On publie le message sur le topic
  brasPublisher.publish(msg);
  //ros::Duration(1).sleep();

}


bool Sauvegarde(){
  ofstream fichier("//usr//users//promo2017//bordogna_rem//catkin_ws//src//blocage_bras//donnees_points.txt", ios::out | ios::app); 

  if(fichier)
    { 
      fichier << lesThetas[0] << ' ' <<  lesThetas[1] << ' ' << lesThetas[2] << ' ' << lesThetas[3] << std::endl;
      fichier.close();
    }
  else{
    cout << "Impossible d'ouvrir le fichier !" << endl;}
}



bool traitementInstruction(char c)
{

  switch (c)
    {//z
    case 122 :
      if(lesThetas[0] + 0.1 > 0.0100692 && lesThetas[0] + 0.1 <  5.84014){
	lesThetas[0] += 0.1;
      }
      else { cout<<"out of range \n";}
      break; //break oblige sinon faire toutes les instructions dessous!!??
      //s
    case 115 :
      if(lesThetas[0] - 0.1 > 0.0100692 && lesThetas[0] - 0.1 <  5.84014){
	lesThetas[0] -= 0.1;
      }
      else { cout<<"out of range \n";}
      break;
      //e
    case 101:
      if(lesThetas[1] + 0.1 > 0.0100692 && lesThetas[1] + 0.1 < 2.61799){
	lesThetas[1] += 0.1;
      }
      else { cout<<"out of range \n";}
      break;
      //d
    case 100 :
      if(lesThetas[1] - 0.1 > 0.0100692 && lesThetas[1] - 0.1 < 2.61799){
	lesThetas[1] -= 0.1;
      }
      break;
      //r
    case 114 :
      if(lesThetas[2] + 0.1 > -5.02655 && lesThetas[2] + 0.1 < -0.015708){
	lesThetas[2] += 0.1;
      }
      else { cout<<"out of range \n";}
      break;
      //f   
    case 102 :
      if(lesThetas[2] - 0.1 > -5.02655 && lesThetas[2] - 0.1 < -0.015708){
	lesThetas[2] -= 0.1;
      }
      else { cout<<"out of range \n";}
      break;
      //t
    case 116 :
      if(lesThetas[3] + 0.1 > 0.0221239 && lesThetas[3] + 0.1 < 3.4292 ){
	lesThetas[3] += 0.1;
      }
      else { cout<<"out of range \n";}
      break;
      //g
    case 103 :
      if(lesThetas[3] - 0.1 > 0.0221239 && lesThetas[3] - 0.1 < 3.4292 ){
	lesThetas[3] -= 0.1;
      }
      else { cout<<"out of range \n";}
      break;

    default    :
      printf("%c[2K", 27); //si rien de bon, on efface la ligne
      cout<<"\n\033[F";//on revient au debut de la ligne a chauque fois

    }

}

void majValeurs()
{
  ROS_INFO("I heard: \n");
}


int main(int argc, char **argv) {
  //Initialisation des valeurs de theta
  lesThetas[0] = 0.111;
  lesThetas[1] = 0.11;
  lesThetas[2] = -0.11;
  lesThetas[3] = 0.11;
  lesThetas[4] = 0.111;

  affichageMessageIntro();
  ecritureTexte();
  char c = 0;
  bool pas_sauvegarde = false;
  ros::init(argc, argv, "enregistrement_pos");
  ros::NodeHandle n;

  //Initialisation des Publishers
  brasPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);//le 1 fait que ca efface l'instructions, le msg publlié des qu'un autre est envoyé au topic
  ros::Rate loop_rate(100);
  sleep(1); //Indispensable pour laisser le temps au bras d'initialiser
  Miseajour();//On publie les modifications
  system("stty raw");

  while (ros::ok()){
    c= getchar();
    if(c==27){ros::shutdown();}
    traitementInstruction(c);//On modifie les angles theta
    Miseajour();//On publie les modifications
    Sauvegarde();//On fait une sauvegarde
  

    //si on sauvergarde "à la main" en tapant espace, instructions ci-dessous:
    //NB : il faut repasser en " cooked"
    /* c= getchar();
       if(c==32){
       pas_sauvegarde = true;
       }
       traitementInstruction(c);//On modifie les angles theta
       Miseajour();//On publie les modifications
       if(pas_sauvegarde){
       Sauvegarde();
       cout << "Position sauvegardée" << endl;
       pas_sauvegarde = false;
       }*/


    loop_rate.sleep();
  }

  system("stty cooked");
  return 0;
}
