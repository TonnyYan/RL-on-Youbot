//Programme qui traite le nuage de points fourni par la Kinect
//pour supprimer les plans principaux
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>

#define DISTANCE_THRESH .08
#define CLOUD_PERCENTAGE .3

ros::Publisher xyz_pub;


//TODO Résoudre problème time stamp

void callback(ros::Publisher& pub, const sensor_msgs::PointCloud2ConstPtr& input) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>),cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::PCLPointCloud2::Ptr  cloud_downsample_in(new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr  cloud_downsample_out(new pcl::PCLPointCloud2 ());
  pcl_conversions::toPCL (*input, *cloud_downsample_in);
  sor.setInputCloud (cloud_downsample_in);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_downsample_out);
  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_downsample_out, *cloud_filtered);
      
  //Segmentation Planaire du nuage de points
  //Variables pour la segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;


  if(not cloud_filtered->points.empty()){//Si nuage non nul


    //Paramétrage de seg
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);//Pour détecter les plans
    seg.setMethodType (pcl::SAC_RANSAC);//Méthode par triplets aléatoires
    seg.setDistanceThreshold (DISTANCE_THRESH);
    seg.setInputCloud (cloud_filtered);//Pb: pcl_cloud doit etre un pointeur !

    //Segmentation
    seg.segment (*inliers, *coefficients);


    //On ne va garder qu'un extrait
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0;

    int nr_points = (int) cloud_filtered->points.size ();

    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > CLOUD_PERCENTAGE* nr_points)
      {
	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0)
	  {
	    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	    break;
	  }
      
	// Extract the inliers
	extract.setInputCloud (cloud_filtered);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_p);

	// Create the filtering object
	extract.setNegative (true);
	extract.filter (*cloud_f);
	cloud_filtered.swap (cloud_f);
	i++;
      }


    //Find the closest point 
    

    if(not cloud_filtered->points.empty()){//Si nuage non nul
      pcl::KdTree<pcl::PointXYZ>::Ptr tree_ (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      tree_->setInputCloud(cloud_filtered);

      std::vector<int> nn_indices (1);
      std::vector<float> nn_dists (1);

      tree_->nearestKSearch(pcl::PointXYZ(0, 0, 0), 1, nn_indices, nn_dists);

      //We publish the closest point coordinates
      geometry_msgs::Point msg;
      msg.x = cloud_filtered->points[nn_indices[0]].x;
      msg.y = cloud_filtered->points[nn_indices[0]].y;
      msg.z = cloud_filtered->points[nn_indices[0]].z;
      xyz_pub.publish(msg);

      // We publish the filtered cloud.
      pcl::PCLPointCloud2 points_out;
      pcl::toPCLPointCloud2(*cloud_filtered,points_out);
      sensor_msgs::PointCloud2 output;
      pcl_conversions::fromPCL(points_out,output);

      tf::TransformBroadcaster broadcaster;

      broadcaster.sendTransform(tf::StampedTransform(tf::Transform(
								   tf::Quaternion(0.5,-0.5,0.5,-0.5),
								   tf::Vector3(0.0,0.0,0.0)),
						     ros::Time(0),
						     "/camera_link",
						     "/base_kinect"));
      //On publie la tf de base_kinect par rapport à camera_link


      output.header.frame_id = "/base_kinect";
      output.header.stamp =ros::Time(0) ;
      pub.publish(output);
    }
  }


  else{//Le nuage est vide, on renvoie un r nul
    geometry_msgs::Point msg;
    msg.x = 0;
    msg.y = 0;
    msg.z = 0;
    xyz_pub.publish(msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "plane_delete");
  ros::NodeHandle n;
  ros::Publisher  pub = n.advertise<sensor_msgs::PointCloud2>("pcl_out", 0);
  xyz_pub = n.advertise<geometry_msgs::Point>("r_min", 1);
  ros::Subscriber sub =n.subscribe<sensor_msgs::PointCloud2>("pcl_in", 1, boost::bind(callback, boost::ref(pub), _1));
  ros::Rate r(10);
  sleep(1);
  while (ros::ok())
    { ros::spinOnce();
      r.sleep();
    }
  return 0;
}
