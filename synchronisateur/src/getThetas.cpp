#include "ros/ros.h"
#include "synchronisateur/getThetas.h"
#include "brics_actuator/JointPositions.h"

ros::Subscriber sub_theta;
std::array<double, 4> theta_courant;

void callback(brics_actuator::JointPositionsConstPtr msg){
  theta_courant =  {
    msg->positions[0].value, 
    msg->positions[1].value, 
    msg->positions[2].value, 
    msg->positions[3].value //mise à jour des thetas courants
  };
}
bool on_request(synchronisateur::getThetas::Request &req,
		synchronisateur::getThetas::Response &res){
  res.theta1 = theta_courant[0];
  res.theta2 = theta_courant[1];
  res.theta3 = theta_courant[2];//renvoi du dernier r
  res.theta4 = theta_courant[3];
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getThetas_serveur");
  ros::NodeHandle n;
  sub_theta = n.subscribe<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1, callback);
  ros::ServiceServer service = n.advertiseService("getThetas", on_request);
  ros::spin();
  return 0;
}
