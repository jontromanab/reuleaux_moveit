#ifndef DATASET_HDF5_H
#define DATASET_HDF5_H

#include "H5Cpp.h"
#include <hdf5.h>
#include<iostream>
#include<sys/stat.h>
#include <unistd.h>
#include <ros/ros.h>
#include<map_generation/WorkSpace.h>
#include<map_generation/utils.h>



namespace reuleaux
{
struct stat st;
typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMapPtr;
typedef std::map< const std::vector< double >*, double > MapVecDoublePtr;
class Hdf5Dataset
{
public:
  Hdf5Dataset(std::string path, std::string filename);
  Hdf5Dataset(std::string fullpath);
  bool saveWorkspaceToMap(const map_generation::WorkSpace& ws);


private:
  bool checkPath(std::string path);
  bool checkfilename(std::string filename);
  void createPath(std::string path);
  bool saveReachMapsToDataset( reuleaux::MultiMapPtr& poses,  reuleaux::MapVecDoublePtr& spheres, float resolution);

  void close();



  std::string path_;
  std::string filename_;
  hid_t file_, group_poses_, group_spheres_, group_capability_;
  hid_t poses_dataset_, sphere_dataset_, capability_dataset_;
  hid_t attr_;
  float res_;


};

}

#endif // DATASET_HDF5_H
