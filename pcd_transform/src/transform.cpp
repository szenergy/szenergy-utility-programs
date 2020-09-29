#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h> 
#include <math.h> 

double translation_x,translation_y,translation_z,roll,pitch,yaw;
std::string source_map;
std::string new_map;

double Convert(double degree) 
{ 
    double pi = 3.14159265359; 
    return (degree * (pi / 180)); 
} 



int
main (int argc, char** argv)
{
  ros::init(argc, argv, "transform_node");
  ros::NodeHandle nh;
  nh.getParam("/pcd_transform/translation_x", translation_x);
  nh.getParam("/pcd_transform/translation_y", translation_y);
  nh.getParam("/pcd_transform/translation_z", translation_z);
  nh.getParam("/pcd_transform/roll", roll);
  nh.getParam("/pcd_transform/pitch", pitch);
  nh.getParam("/pcd_transform/yaw", yaw);
  nh.getParam("/pcd_transform/source_map", source_map);
  nh.getParam("/pcd_transform/new_map", new_map);



  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

  pcl::io::loadPCDFile (source_map, *source_cloud);   //source PCD Map

  // The angle of rotation in radians 
  double theta_x = Convert(roll);                        //roll
  double theta_y = Convert(pitch);                       //pitch  
  double theta_z = Convert(yaw);                         //yaw


  Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();

  transform_2.translation() << translation_x, translation_y, translation_z;   //translation in meters xyz


  Eigen::Matrix3d m;
  m=Eigen::AngleAxisd(theta_x, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(theta_y, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(theta_z, Eigen::Vector3d::UnitZ());
    
  transform_2.rotate(m);

  // transformed_cloud.data.insert(source_cloud)




  // Print the transformation
  printf ("\nTransform Map\n");
  std::cout << transform_2.matrix() << std::endl;


  
  // Executing the transformation
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);  
  pcl::io::savePCDFileASCII(new_map, *transformed_cloud); 

  // Visualization



  return 0;
}