#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::io::loadPCDFile ("/home/autonom/pcd_maps/autoware-200518.pcd", *source_cloud);   //source PCD Map

  // The angle of rotation in radians 
  double theta_x = 0.01229554;                        //roll
  double theta_y= 0.00621208;                        //pitch  
  double theta_z= 1.61596155+0.0471238898038469;     //yaw


  Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();

  transform_2.translation() << 6.97195928e+05, 5.28570419e+06, 1.54990998e+02;   //translation in meters xyz


  Eigen::Matrix3d m;
  m=Eigen::AngleAxisd(theta_x, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(theta_y, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(theta_z, Eigen::Vector3d::UnitZ());
    
  transform_2.rotate(m);


  

  // Print the transformation
  printf ("\nTransform Map\n");
  std::cout << transform_2.matrix() << std::endl;


  
  // Executing the transformation
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);  
  pcl::io::savePCDFile<pcl::PointXYZ>("/home/autonom/pcd_maps/campus_sajat_eltolt_v5.pcd", *transformed_cloud); 

  // Visualization



  return 0;
}