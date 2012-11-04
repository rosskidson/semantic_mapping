#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

void normalEstimation (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn, pcl::PointCloud<
    pcl::PointXYZRGBNormal>::Ptr pointCloudOut)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (pointCloudIn);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.05);
  ne.compute (*pointCloudOut);
  pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
}

int main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "please provide filename as argument" << std::endl;
    exit(0);
  }
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  reader.read (argv[1], *input_cloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud (*input_cloud, *filtered_cloud, indices);

  // calculate normals
  std::cout << "Calculating Normals..." << std::endl;
  normalEstimation (filtered_cloud, normals_cloud);

  pcl::PCDWriter writer;
  writer.write ("output_normals.pcd", *normals_cloud, false);
  return (0);
}
