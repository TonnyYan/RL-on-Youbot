#include "ros/ros.h"
#include "synchronisateur/getR.h"
#include "geometry_msgs/Point.h"

ros::Subscriber sub_rmin;
typedef std::array<double, 3> Rayon; //vecteur à 3 dim r
Rayon r_courant;

void callback(geometry_msgs::Point r){
  r_courant = {r.x, r.y, r.z};
 }

bool on_request(synchronisateur::getR::Request &req,
		synchronisateur::getR::Response &res){
  res.x = r_courant[0];
  res.y = r_courant[1];
  res.z = r_courant[2];
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getR_serveur");
  ros::NodeHandle n;
  sub_rmin = n.subscribe<geometry_msgs::Point>("r_min", 1, callback);
  ros::ServiceServer service = n.advertiseService("getR", on_request);
  ros::spin();
  return 0;
}
