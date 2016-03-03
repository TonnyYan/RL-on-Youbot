//Programme qui traite le nuage de points fourni par la Kinect
//pour supprimer les plans principaux
#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "pcl_ros/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud){
  //Segmentation Planaire du nuage de points
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);//Pour détecter les plans
  seg.setMethodType (pcl::SAC_RANSAC);//Méthode par triplets aléatoires
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "plane_delete");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >("in",1,callback);
  ros::Rate r(10);
  sleep(1);
  while (ros::ok())
    { ros::spinOnce();
      r.sleep();
    }
  return 0;
}
