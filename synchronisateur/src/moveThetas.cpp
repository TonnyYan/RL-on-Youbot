//On envoie une requete de mouvement de bras, le service le transmet à blocage_bras qui l'effectue (ou non), quand le bras ne bouge plus le service envoie une réponse pour dire qu'il est immobile

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "synchronisateur/moveThetas.h"
#include "brics_actuator/JointPositions.h"

ros::Subscriber sub_bras;
ros::Publisher pub_bras;
bool isMoving;


void callback(brics_actuator::JointPositionsConstPtr msg){
  if(true ) //Condition sur velocity ??)--> Utiliser /joint_states !!
    { isMoving = false;
    }
}


bool on_request(synchronisateur::moveThetas::Request &req,
		synchronisateur::moveThetas::Response &res){
  std::array<double, 5> jointvalues;
  //On a une demande de mouvement de bras
  jointvalues = {req.theta1,req.theta2,req.theta3,req.theta4,0};
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
  pub_bras.publish(msg);
  isMoving = true;
  ros::Duration(0.5).sleep();//On attend que le bras se soit mis en mvt pour vérifier
  while(isMoving){
    ros::spinOnce();
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveThetas_serveur");
  ros::NodeHandle n;
  sub_bras = n.subscribe<brics_actuator::JointPositions>("/arm_1/arm_controller/velocity_command", 1, callback);
  pub_bras = n.advertise<brics_actuator::JointPositions>("out", 1);
  ros::ServiceServer service = n.advertiseService("moveTheta", on_request);
  isMoving = false;
  ros::spin();
  return 0;
}
