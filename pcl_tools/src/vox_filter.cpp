#include <pcl_typedefs/pcl_typedefs.h>
#include "pcl_tools/pcl_tools.h"

#include <iostream>
#include <pcl/io/pcd_io.h>

int
main (int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "please provide filename followed by leaf size (e.g. cloud.pcd 0.01)" << std::endl;
    exit(0);
  }
  PointCloudPtr input_cloud_ptr (new PointCloud);
  PointCloudPtr output_cloud_ptr;
  float leaf_size = atof(argv[2]);

  //Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (argv[1], *input_cloud_ptr); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << input_cloud_ptr->width * input_cloud_ptr->height
       << " data points (" << pcl::getFieldsList (*input_cloud_ptr) << ").";

  output_cloud_ptr = pcl_tools::downsampleCloud(input_cloud_ptr, leaf_size);

  std::cerr << "PointCloud after filtering: " << output_cloud_ptr->width * output_cloud_ptr->height
       << " data points (" << pcl::getFieldsList (*output_cloud_ptr) << ").";

  pcl::PCDWriter writer;
  writer.write ("filter_out.pcd", *output_cloud_ptr,true);

  return (0);
}
