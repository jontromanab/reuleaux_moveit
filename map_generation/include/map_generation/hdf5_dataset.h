#ifndef HDF5_DATASET_H
#define HDF5_DATASET_H
#include "H5Cpp.h"
#include <hdf5.h>
#include<iostream>
#include<sys/stat.h>
#include <unistd.h>
#include <ros/ros.h>
#include<map_generation/WorkSpace.h>
#include<map_generation/utility.h>



namespace reuleaux
{
struct stat st;

class Hdf5Dataset
{
public:
  Hdf5Dataset(std::string path, std::string filename);
  Hdf5Dataset(std::string fullpath);
  bool save(const map_generation::WorkSpace& ws);
  bool load(map_generation::WorkSpace& ws);



private:
  bool checkPath(std::string path);
  bool checkfilename(std::string filename);
  void createPath(std::string path);
  bool saveMap(const reuleaux::VecVecDouble& poses, const reuleaux::VecVecDouble& spheres, const reuleaux::VecDouble& ri, const double resolution);
  bool saveWorkspaceToMap(const map_generation::WorkSpace& ws);

  void close();
  bool open();
  bool getResolution(float& resolution);
  bool getMultimap(reuleaux::MultiMap& mMap);
  bool getSphereRI(reuleaux::MapVecDouble& mvec);
  bool getWorkspace(map_generation::WorkSpace& ws);



  std::string path_;
  std::string filename_;
  hid_t file_, group_poses_, group_spheres_, group_capability_;
  hid_t poses_dataset_, sphere_dataset_, capability_dataset_;
  hid_t attr_;
  float res_;
  map_generation::WorkSpace ws_;
  reuleaux::MultiMap mMap_;
  reuleaux::MapVecDouble mvec_;


};

}

#endif // HDF5_DATASET_H
