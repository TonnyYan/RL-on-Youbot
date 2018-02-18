//On envoie une requete de mouvement de bras, le service le transmet à blocage_bras qui l'effectue (ou non), quand le bras ne bouge plus le service envoie une réponse pour dire qu'il est immobile

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "synchronisateur/moveThetas.h"
#include "sensor_msgs/JointState.h"
#include "brics_actuator/JointPositions.h"


ros::Subscriber sub_bras;
ros::Publisher pub_bras;
bool isMoving;


void callback(sensor_msgs::JointStateConstPtr msg){
  if(
     abs(msg->velocity[0]) == 0.0 &&
     abs(msg->velocity[1]) == 0.0 &&
     abs(msg->velocity[2]) == 0.0 &&
     abs(msg->velocity[3]) == 0.0
     ) {isMoving = false;}
}


bool on_request(synchronisateur::moveThetas::Request &req,
		synchronisateur::moveThetas::Response &res){
  std::array<double, 5> jointvalues;
  //On a une demande de mouvement de bras
  jointvalues = {req.theta1,req.theta2,req.theta3,req.theta4,0.111};
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
  pub_bras.publish(msg);
  ros::Duration(1).sleep();//On attend que le bras se soit mis en mvt pour vérifier
  isMoving = true;
  while(isMoving){
    ros::spinOnce();
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveThetas_serveur");
  ros::NodeHandle n;
  sub_bras = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, callback);
  pub_bras = n.advertise<brics_actuator::JointPositions>("out", 1);
  ros::ServiceServer service = n.advertiseService("moveThetas", on_request);
  isMoving = false;
  ros::spin();
  return 0;
}
