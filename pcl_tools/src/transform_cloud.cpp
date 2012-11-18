#include <iostream>
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZ PointType;

int main (int argc, char** argv)
{
  if (argc != 3)
  {
    ROS_INFO_STREAM("please provide a pointcloud file followed by a text file containing a transformation matrix as arguments");
    exit(0);
  }
  pcl::PCDReader reader;
  pcl::PointCloud<PointType>::Ptr cloudIn (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloudOut (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloudOut_inv (new pcl::PointCloud<PointType>);
  Eigen::Matrix4f trafo, trafo_inv;
  reader.read (argv[1], *cloudIn);
  std::ifstream myfile;
  myfile.open (argv[2]);
  for (int row = 0; row < 4; row++)
    for (int col = 0; col < 4; col++)
    {
      myfile >> trafo (row, col);
    }
  trafo_inv = trafo.inverse();
  ROS_INFO_STREAM("transform to be used: \n" << trafo);


  transformPointCloud (*cloudIn, *cloudOut, trafo);
  transformPointCloud (*cloudIn, *cloudOut_inv, trafo_inv);

  pcl::PCDWriter writer;
  writer.write ("output.pcd", *cloudOut, false);
  writer.write ("output_inverse.pcd", *cloudOut_inv, false);
  return (0);
}
