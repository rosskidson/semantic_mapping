#!/bin/bash
 #	This script will search and replace all instances of pcl with pcl17 for the fuerte pcl17 namespace hack
 # Author: Ross Kidson

 for f in `find . -name \*.cpp -o -name \*.h -o -name \*.hpp`;
 do
 if egrep -rc 'pcl::|namespace\ pcl|pcl_gpu|pcl_cuda|include\ <pcl\/|include\ "pcl\/|pcl_round|pcl_lrint|pcl_sleep|PCLAPI|aligned_malloc|aligned_free|POINT_CLOUD_' $f > 0 ;
 then
  echo "Instances of pcl found in $f, replacing with pcl17"
  sed -i 's/pcl::/pcl17::/g' $f
  sed -i 's/namespace\ pcl/namespace\ pcl17/g' $f
  sed -i 's/pcl_gpu::/pcl_gpu17::/g' $f
  sed -i 's/pcl_cuda::/pcl_cuda17::/g' $f
  sed -i 's/include\ <pcl\//include <pcl17\//g' $f
  sed -i 's/include\ "pcl\//include "pcl17\//g' $f
  sed -i 's/pcl_round/pcl17_round/g' $f
  sed -i 's/pcl_lrint/pcl17_lrint/g' $f
  sed -i 's/pcl_sleep/pcl17_sleep/g' $f
  sed -i 's/PCLAPI/PCL17API/g' $f
  sed -i 's/PCL_ERROR/PCL17_ERROR/g' $f
  sed -i 's/aligned_malloc/pcl17_aligned_malloc/g' $f
  sed -i 's/aligned_free/pcl17_aligned_free/g' $f
  sed -i 's/POINT_CLOUD_/PCL17_POINT_CLOUD_/g' $f
 fi
done

for f in `find ./ -name CMakeLists.txt`;
do
  echo "Replacing pcl with pcl17 in CMakeLists.txt file  $f"
  sed -i 's/include\/pcl/include\/pcl17/g' $f
done
