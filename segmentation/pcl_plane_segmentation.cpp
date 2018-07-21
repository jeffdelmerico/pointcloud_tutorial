#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_segmented (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("../data/table_scene_lms400.pcd", *cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  // Segment dominant plane
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloud_segmented);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  PCL_INFO("Saving dominant plane in input cloud to: table_scene_lms400_first_plane.pcd\n");
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_first_plane.pcd", *cloud_segmented, false);
  
  // Remove inliers from input and repeat for 2nd dominant plane
  pcl::ExtractIndices<pcl::PointXYZ> extract; 
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*outliers);

  // Run segmentation on outliers
  seg.setInputCloud (outliers);
  seg.segment (*inliers, *coefficients);
  pcl::copyPointCloud<pcl::PointXYZ>(*outliers, *inliers, *outliers_segmented);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  PCL_INFO("Saving dominant plane in outliers to: table_scene_lms400_second_plane.pcd\n");
  writer.write<pcl::PointXYZ> ("table_scene_lms400_second_plane.pcd", *outliers_segmented, false);

  return (0);
}
