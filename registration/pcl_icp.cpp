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

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud (cloud2);
  icp.setInputTarget (cloud1);
  icp.setMaximumIterations (20);
  icp.setMaxCorrespondenceDistance (0.1);
  Eigen::Matrix4f trafo;
  icp.align (*cloud2);
  (*cloud2) += *(cloud1);

  pcl::io::savePCDFile ("icp_aligned.pcd", *cloud2);
  std::cout << "Converged: " << (icp.hasConverged() ? "True" : "False") << " Score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << "Transformation matrix:" << std::endl << icp.getFinalTransformation() << std::endl;

  return 0;
}
