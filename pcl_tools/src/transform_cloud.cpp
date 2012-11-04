#include <iostream>
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

int main (int argc, char** argv)
{
  if (argc != 3)
  {
    ROS_INFO_STREAM("please provide a pointcloud file followed by a text file containing a transformation matrix as arguments");
    exit(0);
  }
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Matrix4f trafo;
  reader.read (argv[1], *cloudIn);
  std::ifstream myfile;
  myfile.open (argv[2]);
  for (int row = 0; row < 4; row++)
    for (int col = 0; col < 4; col++)
    {
      myfile >> trafo (row, col);
    }

  transformPointCloud (*cloudIn, *cloudOut, trafo);

  pcl::PCDWriter writer;
  writer.write ("output.pcd", *cloudOut, false);
  return (0);
}
