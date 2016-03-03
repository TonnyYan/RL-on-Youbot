//Programme qui traite le nuage de points fourni par la Kinect
//pour supprimer les plans principaux

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

ros::Subscriber sub;

void callback(const pcl::PointCloud& cloud){
 


    }

int main(int argc, char **argv) {
  ros::init(argc, argv, "plane_delete");
  ros::NodeHandle n;

  sub = n.subscribe("in",1,callback);
  ros::Rate loop_rate(10);
  sleep(1);
  while (ros::ok())
    { ros::spinOnce();
      r.sleep();
    }
  return 0;
}
