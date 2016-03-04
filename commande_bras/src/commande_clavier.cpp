#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
#include <curses.h>
#include <iostream>
#include <sstream>
#include <array>
#include <iterator>
#include <string>

ros::Publisher brasPublisher;
ros::Publisher pincePublisher;
ros::Publisher platformPublisher;
ros::Subscriber pinceSubscriber;



void affichageMessageIntro()
{


  std::string messageIntro("");
  messageIntro = "Bienvenue dans le manipulateur youbot \n \nCommandez le robot avec les touches suivantes : \n " ;
  messageIntro += "\n|  z : avancer              |"; //
  messageIntro += "\n|  s : reculer              |";
  messageIntro += "\n|  q : gauche               |";
  messageIntro += "\n|  d : droite               |";
  messageIntro += "\n|  espace : stop            |";
  messageIntro += "\n|  e : rotation droite      |";
  messageIntro += "\n|  a : rotation gauche      | \n";
  messageIntro += "\n|  p : ouverture pince      |";
  messageIntro += "\n|  m : fermeture pince      | \n ";
  messageIntro += "\n|  c : tourner bras à gauche|";
  messageIntro += "\n|  v : tourner bras à droite| \n";
  messageIntro += "\n|  x : déplier le bras      | \n";
  std::cout << messageIntro;

}



void bougerBras(int i) {

  //Initialisation des valeurs
  std::array<double,5> jointvalues;



  // jointvalues[0] = 2.95;//Rotation de la base
  // jointvalues[1] = 0.11;//Valeurs par défaut de calibration
  // jointvalues[2] = -0.11;
  // jointvalues[3] = 0.11;
  // jointvalues[4] = 0.111;
    

  switch (i) {
  
  case -1 : jointvalues = {2.95, 0.11, -0.11, 0.11, 0.111}; break;
  case  1 :
    jointvalues[0] = 0.111;//Rotation de la base
    jointvalues[1] = 0.11;//Valeurs par défaut de calibration
    jointvalues[2] = -0.11;
    jointvalues[3] = 0.11;
    jointvalues[4] = 0.111;
    break;
  case 0 :
    jointvalues[0] = 2.95;//Dépliement du bras
    jointvalues[1] = 1.05;
    jointvalues[2] = -2.44;
    jointvalues[3] = 1.73;
    jointvalues[4] = 2.95;
    break;
  }


  //Création de messages positions
  //Contient une pile de 5 messages de type JointValue
  brics_actuator::JointPositions msg;
  auto out = std::back_inserter(msg.positions);

  brics_actuator::JointValue joint;
  joint.unit = boost::units::to_string(boost::units::si::radian);
  joint.timeStamp = ros::Time::now();
  unsigned int joint_rank = 0;
  for(auto& value : jointvalues) {
    joint.value = value;
    joint.joint_uri = "arm_joint_" + std::to_string(joint_rank++);
    *(out++) = joint;
  }
  
  //On publie le message sur le topic
  brasPublisher.publish(msg);
  //ros::Duration(1).sleep();

}

void bougerPince(int sens){

  brics_actuator::JointPositions msg;
  brics_actuator::JointValue joint;

  joint.timeStamp = ros::Time::now();
  joint.unit = boost::units::to_string(boost::units::si::meter);
  if (sens == 1){ joint.value = 0.011;} //on ouvre
  if (sens == -1){ joint.value = 0.001;}  //on ferme

  joint.joint_uri = "gripper_finger_joint_l";
  msg.positions.push_back(joint);
  joint.joint_uri = "gripper_finger_joint_r";
  msg.positions.push_back(joint);

  pincePublisher.publish(msg);
  //ros::Duration(1).sleep();//Sleep -> Indispensable pour prise en compte commande
  //cette commande n'est plus necessaire!

}



void bougerPlatform(int i)
{
  geometry_msgs::Twist twist;

  switch (i)
    {//z
    case 122 :
      twist.linear.x = 0.15;
      twist.linear.y = 0;
      twist.angular.x = 0;
      break;
      //s
    case 115 :
      twist.linear.x = -0.15;
      twist.linear.y = 0;
      twist.angular.x = 0;
      break;
      //q
    case 113 :
      twist.linear.x = 0;
      twist.linear.y = 0.15;
      twist.angular.x = 0;
      break;
      //d
    case  100:
      twist.linear.x = 0;
      twist.linear.y = -0.15;
      twist.angular.x = 0;
      break;
      //e
    case 101 :
      twist.linear.x = 0;
      twist.linear.y = 0;
      twist.angular.z = 0.15;
      break;
    case 97 :
      twist.linear.x = 0;
      twist.linear.y = 0;
      twist.angular.z = -0.15;
      break;
      //espace
    case 32 :
      twist.linear.x = 0;
      twist.linear.y = 0;
      twist.angular.x = 0;
      break;
    }
  platformPublisher.publish(twist);
  //  ros::Duration(0).sleep();
}


bool traitementInstruction(char c)
{

  switch (c)
    {//z
    case 122 :
      bougerPlatform(122);
      break; //break oblige sinon faire toutes les instructions dessous!!??
      //s
    case 115 :
      bougerPlatform(115);
      break;
      //q
    case 113:
      bougerPlatform(113);
      break;
      //d
    case 100 :
      bougerPlatform(100);
      break;
      //e
    case 101 :
      bougerPlatform(101);
      break;
    case 97 :
      bougerPlatform(97);
      break;
      //p
    case 112 :
      bougerPince(1);
      break;
      //m
    case 109 :
      bougerPince(-1);
      break;
      //c
    case 99 :
      bougerBras(-1);
      break;
      //v
    case 118 :
      bougerBras(1);
      break;
      //x
    case 120 :
      bougerBras(0);
      break;
      //espace
    case ' ' :
      bougerPlatform(32);
      break;
    default    :
      std::cout<< "\027[2K" << "\n\033[F" << std::flush; //on revient au debut de la ligne a chauque fois

    }

}

void majValeurs()
{
  ROS_INFO("I heard: \n");
}


int main(int argc, char **argv) {

  affichageMessageIntro();
  char c = 0;
  system("stty raw");//terminal lit direct ce qui rentre system("stty cooked"); pour revenir a la normale

  ros::init(argc, argv, "commande_clavier");
  ros::NodeHandle n;

  //Initialisation des Publishers
  brasPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);//le 1 fait que ca efface l'instructions, le msg publlié des qu'un autre est envoyé au topic
  pincePublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
  platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  ros::Rate loop_rate(10);
  ros::Duration(1).sleep(); //Indispensable pour laisser le temps au bras d'initialiser

  while (ros::ok()){
    c= getchar();
    if(c==27){ros::shutdown();}
    traitementInstruction(c);
    loop_rate.sleep();
  }
  system("stty cooked");
  return 0;
}



