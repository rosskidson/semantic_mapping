#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include "pcl_tools/pcl_tools.h"


int main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "please provide filename as argument" << std::endl;
    exit(0);
  }
  pcl::PCDReader reader;
  PointCloudPtr input_cloud_ptr (new PointCloud);
  PointCloudPtr filtered_cloud_ptr (new PointCloud);
  PointCloudNormalsPtr normals_cloud_ptr (new PointCloudNormals);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  reader.read (argv[1], *input_cloud_ptr);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud (*input_cloud_ptr, *filtered_cloud_ptr, indices);

  // calculate normals
  std::cout << "Calculating Normals..." << std::endl;
  pcl_tools::calculateNormalsOMP(filtered_cloud_ptr, normals_cloud_ptr);
  for(int i = 0; i<filtered_cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZRGBNormal new_point;
    new_point.x = filtered_cloud_ptr->points[i].x;
    new_point.y = filtered_cloud_ptr->points[i].y;
    new_point.z = filtered_cloud_ptr->points[i].z;
    //new_point.rgb = filtered_cloud_ptr->points[i].rgb;
    new_point.normal_x = normals_cloud_ptr->points[i].normal_x;
    new_point.normal_y = normals_cloud_ptr->points[i].normal_y;
    new_point.normal_z = normals_cloud_ptr->points[i].normal_z;
    output_cloud_ptr->points.push_back(new_point);
  }

  output_cloud_ptr->width = output_cloud_ptr->points.size();
  output_cloud_ptr->height = 1;

  pcl::PCDWriter writer;
  writer.write ("output_normals.pcd", *output_cloud_ptr, false);
  return (0);
}
