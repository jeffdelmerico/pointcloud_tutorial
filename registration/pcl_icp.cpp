#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <iostream>

int
main (int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile (argv[1], *cloud1) == -1)
  {
    std::cout << "Could not read file" << std::endl;
    return -1;
  }
  std::cout << "width: " << cloud1->width << " height: " << cloud1->height << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile (argv[2], *cloud2) == -1)
  {
      std::cout << "Could not read file" << std::endl;
      return -1;
  }
  std::cout << "width: " << cloud2->width << " height: " << cloud2->height << std::endl;
  Eigen::Matrix4f trafo;
  trafo << 0.862, 0.011, -0.507,  0.5,
          -0.139, 0.967, -0.215,  0.7,
           0.487, 0.255,  0.835, -1.4,
           0.0,   0.0,    0.0,    1.0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_init (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud (*cloud1, *cloud1_init, trafo);

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud (cloud2);
  icp.setInputTarget (cloud1_init);
  icp.setMaximumIterations (30);
  icp.setMaxCorrespondenceDistance (0.02);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZRGB>);
  icp.align (*aligned);
  (*aligned) += *(cloud1_init);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr init (new pcl::PointCloud<pcl::PointXYZRGB>);
  (*init) = (*cloud1_init) + (*cloud2);
  pcl::io::savePCDFile ("../data/pcl_icp_aligned.pcd", *aligned);
  pcl::io::savePCDFile ("../data/pcl_icp_init.pcd", *init);
  std::cout << "Converged: " << (icp.hasConverged() ? "True" : "False") << " Score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << "Transformation matrix:" << std::endl << icp.getFinalTransformation() << std::endl;

  return 0;
}
