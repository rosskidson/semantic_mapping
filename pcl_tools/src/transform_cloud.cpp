#include "pcl_tools/pcl_tools.h"

#include <iostream>
#include <pcl/io/pcd_io.h>

int main (int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "please provide a pointcloud file followed by a text file containing a transformation matrix as arguments\n";
    exit(0);
  }
  pcl::PCDReader reader;
  PointCloudPtr input_cloud_ptr (new PointCloud);
  PointCloudPtr output_cloud_ptr (new PointCloud);
  PointCloudPtr inv_cloud_ptr (new PointCloud);

  Eigen::Matrix4f trafo, trafo_inv;
  reader.read (argv[1], *input_cloud_ptr);
  std::ifstream myfile;
  myfile.open (argv[2]);
  for (int row = 0; row < 4; row++)
    for (int col = 0; col < 4; col++)
    {
      myfile >> trafo (row, col);
    }
  trafo_inv = trafo.inverse();

  std::cerr << "transform to be used: \n" << trafo;
  pcl_tools::transformPointCloud(input_cloud_ptr, output_cloud_ptr, trafo);
  pcl_tools::transformPointCloud(input_cloud_ptr, inv_cloud_ptr, trafo_inv);

  pcl::PCDWriter writer;
  writer.write ("output.pcd", *output_cloud_ptr, false);
  writer.write ("output_inverse.pcd", *inv_cloud_ptr, false);
  return (0);
}
