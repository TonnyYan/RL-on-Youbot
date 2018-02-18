#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/String.h>
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
using namespace std;


ros::Subscriber armSubscriber;

void callback(const sensor_msgs::JointState& msg)
{
  /*uint32 seq
  time stamp
  string frame_id
string[] name
float64[] position
float64[] velocity
float64[] effort*/
ROS_INFO_STREAM("maj enregistrement donnes debut");
std::vector<string> nom = msg.name; //taille 12 mais seul 7 ont un vrai truc, les derniers sont a null
std::vector<double> vel = msg.velocity;//idem
std::vector<double> pos = msg.position;//idem

for(int i(0);i<7;i++){
  ROS_INFO_STREAM(nom[i]<<" = "<<vel[i]<<"position = "<<pos[i]);
}

ROS_INFO_STREAM("maj enregistrement donnes fin");
}



int main(int argc, char **argv) {

	ros::init(argc, argv, "commande_clavier_2");
	ros::NodeHandle n;
	
	//Initialisation des Subscribers
	armSubscriber = n.subscribe("joint_states",1,callback);
	ros::Rate r(1);

	while (ros::ok())
	{ ros::spinOnce();
		 r.sleep();
	}
	
	 
        return 0;
}



