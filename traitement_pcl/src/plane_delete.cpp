//Programme qui traite le nuage de points fourni par la Kinect
//pour supprimer les plans principaux
#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>

void callback(ros::Publisher& pub,
	      const sensor_msgs::PointCloud2ConstPtr& input) {
  //Conversion
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);
  
  //Segmentation Planaire du nuage de points
  //Variables pour la segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  //Paramétrage de seg
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);//Pour détecter les plans
  seg.setMethodType (pcl::SAC_RANSAC);//Méthode par triplets aléatoires
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud);//Pb: pcl_cloud doit etre un pointeur !

  //Segmentation
  seg.segment (*inliers, *coefficients);

  std::cout << "Model coefficients: " << coefficients->values[0] << " " 
	    << coefficients->values[1] << " "
	    << coefficients->values[2] << " " 
	    << coefficients->values[3] << std::endl;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "plane_delete");
  ros::NodeHandle n;
  ros::Publisher  pub = n.advertise<sensor_msgs::PointCloud2>("pcl_out", 0);
  ros::Subscriber sub =n.subscribe<sensor_msgs::PointCloud2>("pcl_in", 1, boost::bind(callback, boost::ref(pub), _1));
  ros::Rate r(10);
  sleep(1);
  while (ros::ok())
    { ros::spinOnce();
      r.sleep();
    }
  return 0;
}
