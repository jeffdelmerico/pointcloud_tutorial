#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
main (int argc, char** argv)
{
    if (argc != 2)
    {
        PCL_ERROR("Usage: %s mode --- mode 0 is pass through\n\t\t\t    mode 1 is voxel grid\n\t\t\t    mode 2 is statistical outlier removal\n", argv[0]);
        return(-1);
    }
    // Switch modes
    int mode = atoi(argv[1]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read ("../data/table_scene_lms400.pcd", *cloud);

    PCL_INFO("PointCloud before filtering: %d data points.\n", cloud->width * cloud->height);

    if(mode == 0)
    {
        // PassThrough filter
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-0.75, 0.5);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_filtered);
    }
    else if(mode == 1)
    {
        // Downsample to voxel grid
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);
    }
    else
    {
        // Statistical Outlier Removal
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_filtered);
    }

    PCL_INFO("PointCloud before filtering: %d data points.\n", cloud_filtered->width * cloud_filtered->height);

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_filtered.pcd", *cloud_filtered, false);

    return (0);
}
