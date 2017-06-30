#include<map_generation/dataset_hdf5.h>

namespace reuleaux
{
Hdf5Dataset::Hdf5Dataset(std::string fullpath)
{
  std::stringstream fp(fullpath);
  std::string segment;
  std::vector<std::string> seglist;
  char the_path[256];

  while(std::getline(fp, segment, '/'))
  {
     seglist.push_back(segment);
  }

  std::ostringstream oss_file;
  oss_file<< seglist.back();
  this->filename_ = oss_file.str();


  seglist.pop_back();
  std::ostringstream oss_path;
  if (!seglist.empty())
  {
    std::copy(seglist.begin(), seglist.end()-1,
    std::ostream_iterator<std::string>(oss_path, "/"));
    oss_path << seglist.back();
    oss_path<<"/";
  }
  else
  {
    getcwd(the_path, 255);
    strcat(the_path, "/");
    oss_path << the_path;
  }

 this->path_ = oss_path.str();
 checkPath(this->path_);
 checkfilename(this->filename_);

}

Hdf5Dataset::Hdf5Dataset(std::string path, std::string filename)
{
  this->path_ = path;
  this->filename_ = filename;
  checkPath(this->path_);
  checkfilename(this->filename_);
}

bool Hdf5Dataset::checkPath(std::string path)
{
  if (stat(path.c_str(), &st)!=0)
  {
    ROS_INFO("Path does not exist yet");
  }
}

bool Hdf5Dataset::checkfilename(std::string filename)
{
  std::string ext = ".h5";
  if(filename.find(ext) == std::string::npos)
  {
    ROS_ERROR("Please provide an extension of .h5 It will make life easy");
    exit(1);
  }
}

void Hdf5Dataset::createPath(std::string path)
{
  ROS_INFO("Creating Directory");
  const int dir_err = mkdir(this->path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if(1 == dir_err)
  {
    ROS_INFO("Error Creating Directory");
    exit(1);
  }
}

void Hdf5Dataset::close()
{
  H5Aclose(this->attr_);
  H5Dclose(this->poses_dataset_);
  H5Gclose(this->group_poses_);
  H5Dclose(this->sphere_dataset_);
  H5Gclose(this->group_spheres_);
  H5Fclose(this->file_);
}

bool Hdf5Dataset::saveReachMapsToDataset(reuleaux::MultiMapPtr &poses, reuleaux::MapVecDoublePtr &spheres, float resolution)
{
  if(!checkPath(this->path_))
  {
    createPath(this->path_);
  }

  //Creating Multimap to Straight vector<vector<double> > //Can take this function to a new class
  std::vector< std::vector< double > > pose_reach;
  for (MultiMapPtr::iterator it = poses.begin(); it != poses.end(); ++it)
  {
    const std::vector<double>* sphere_coord    = it->first;
    const std::vector<double>* point_on_sphere = it->second;
    std::vector< double > pose_and_sphere(10);
    //pose_and_sphere.reserve( sphere_coord->size() + point_on_sphere->size());
    for (int i = 0; i < 3; i++)
      {
        pose_and_sphere[i]=((*sphere_coord)[i]);
      }
    for (int j = 0; j < 7; j++)
      {
        pose_and_sphere[3+j] = ((*point_on_sphere)[j]);
      }
    pose_reach.push_back(pose_and_sphere);
   }

  const char *filepath = this->path_.c_str();
  const char *name = this->filename_.c_str();
  char fullpath[100];
  strcpy(fullpath, filepath);
  strcat(fullpath, name);
  ROS_INFO("Saving map %s", this->filename_.c_str());
  this->file_ = H5Fcreate(fullpath, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  this->group_poses_ = H5Gcreate(this->file_, "/Poses", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  this->group_spheres_ = H5Gcreate(this->file_, "/Spheres", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  ROS_INFO("Saving poses in reachability map");
  const hsize_t ndims = 2;
  const hsize_t ncols = 10;

  int posSize = poses.size();
  int chunk_size;
  int PY = 10;
  if (posSize % 2)
      {
        chunk_size = (posSize / 2) + 1;
      }
  else
      {
        chunk_size = (posSize / 2);
      }
  // Create Dataspace
  hsize_t dims[ndims] = {0, ncols};  // Starting with an empty buffer
  hsize_t max_dims[ndims] = {H5S_UNLIMITED, ncols};  // Creating dataspace
  hid_t file_space = H5Screate_simple(ndims, dims, max_dims);

  // Create Dataset Property list
  hid_t plist = H5Pcreate(H5P_DATASET_CREATE);
  H5Pset_layout(plist, H5D_CHUNKED);
  hsize_t chunk_dims[ndims] = {chunk_size, ncols};
  H5Pset_chunk(plist, ndims, chunk_dims);


  // Create the datset
  this->poses_dataset_ = H5Dcreate(this->group_poses_, "poses_dataset", H5T_NATIVE_FLOAT, file_space, H5P_DEFAULT, plist, H5P_DEFAULT);
  // Closing resources
  H5Pclose(plist);
  H5Sclose(file_space);

  // Creating the first buffer
  hsize_t nlines = chunk_size;
  float *buffer = new float[nlines * ncols];
  float **dset1_data = new float *[nlines];
  for (hsize_t i = 0; i < nlines; ++i)
    {
      dset1_data[i] = &buffer[i * ncols];
    }

  // Data for the first chunk
  for (int i = 0; i < chunk_size; i++)
    {
      for (int j = 0; j < PY; j++)
      {
        dset1_data[i][j] = pose_reach[i][j];
      }
    }
  // Memory dataspace indicating size of the buffer
  dims[0] = chunk_size;
  dims[1] = ncols;
  hid_t mem_space = H5Screate_simple(ndims, dims, NULL);

  // Extending dataset
  dims[0] = chunk_size;
  dims[1] = ncols;
  H5Dset_extent(this->poses_dataset_, dims);

  // Selecting hyperslab on the dataset
  file_space = H5Dget_space(this->poses_dataset_);
  hsize_t start[2] = {0, 0};
  hsize_t count[2] = {chunk_size, ncols};
  H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

  // Writing buffer to the dataset
  H5Dwrite(this->poses_dataset_, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

  // Closing file dataspace
  H5Sclose(file_space);
  // Data for the Second chunk
  for (int i = chunk_size; i < posSize; i++)
    {
      for (int j = 0; j < PY; j++)
      {
        dset1_data[i - chunk_size][j] = pose_reach[i][j];
      }
    }

  // Resizing new memory dataspace indicating new size of the buffer
  dims[0] = posSize - chunk_size;
  dims[1] = ncols;
  H5Sset_extent_simple(mem_space, ndims, dims, NULL);

  // Extend dataset
  dims[0] = posSize;
  dims[1] = ncols;
  H5Dset_extent(this->poses_dataset_, dims);
  // Selecting hyperslab
  file_space = H5Dget_space(this->poses_dataset_);
  start[0] = chunk_size;
  start[1] = 0;
  count[0] = posSize - chunk_size;
  count[1] = ncols;
  H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

  // Writing buffer to dataset
  H5Dwrite(this->poses_dataset_, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

  // Closing all the resources
  delete[] dset1_data;
  delete[] buffer;


  // Creating Sphere dataset
  ROS_INFO("Saving spheres in Reachability map");
  hid_t sphere_dataspace;
  const int SX = spheres.size();
  const int SY = 4;

  hsize_t dims2[2];  // dataset dimensions
  dims2[0] = SX;
  dims2[1] = SY;
  double dset2_data[SX][SY];

  for (MapVecDoublePtr::iterator it =  spheres.begin(); it !=spheres.end(); ++it)
    {
      for (int j = 0; j < SY - 1; j++)
      {
        dset2_data[distance( spheres.begin(), it)][j] = (*it->first)[j];
      }
      for (int j = 3; j < SY; j++)
      {
        dset2_data[distance( spheres.begin(), it)][j] = it->second;
      }
    }
  sphere_dataspace = H5Screate_simple(2, dims2, NULL);
  this->sphere_dataset_ = H5Dcreate2(this->group_spheres_, "sphere_dataset", H5T_NATIVE_DOUBLE,
                                     sphere_dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(this->sphere_dataset_, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, dset2_data);

  // Creating attribute


  hsize_t attr_dims;
  float attr_data[1];
  attr_data[0] = resolution;
  attr_dims = 1;
  sphere_dataspace = H5Screate_simple(1, &attr_dims, NULL);
  this->attr_ = H5Acreate2(this->sphere_dataset_, "Resolution", H5T_NATIVE_FLOAT, sphere_dataspace,
                           H5P_DEFAULT, H5P_DEFAULT);
  H5Awrite(this->attr_, H5T_NATIVE_FLOAT, attr_data);
  //H5Aclose(this->attr_);

  // Closing all

  H5Sclose(sphere_dataspace);
  H5Sclose(file_space);
  H5Sclose(mem_space);
  close();
}

/*bool Hdf5Dataset::saveWorkspaceToMap(const map_generation::WorkSpace &ws)
{
  reuleaux::MultiMapPtr pose_col_filter;
  reuleaux::MapVecDoublePtr sphere_color;
  float res = ws.resolution;
  for(int i=0;i<ws.WsSpheres.size();++i)
  {
    std::vector<double> sphere_vec = reuleaux::pointToVector(ws.WsSpheres[i].point);
    //std::vector<double>* sphere_vec = new std::vector<double>(3);
    //reuleaux::pointToVector(ws.WsSpheres[i].point, sphere_vec);
    float ri = ws.WsSpheres[i].ri;
    std::cout<<"Inserting: "<<sphere_vec[0]<<" "<<sphere_vec[1]<<" "<<sphere_vec[2]<<std::endl;
    sphere_color.insert(std::make_pair(&sphere_vec, double(ri)));
    std::cout<<"processing sphere: "<<i+1<<std::endl;
    //std::cout<<"size of sphere color"<<sphere_color.size()<<std::endl;
    /*for(int j=0;ws.WsSpheres[i].poses.size();++j)
    {
      std::vector<double> pose_vec(7);
      reuleaux::poseToVector(ws.WsSpheres[i].poses[j], pose_vec);
      //std::cout<<"size of pose_vec: "<<pose_vec.size()<<std::endl;
      pose_col_filter.insert(std::make_pair(&sphere_vec, &pose_vec));
      //std::cout<<"size of POSECOLFILTER color"<<pose_col_filter.size()<<std::endl;

  }

  std::cout<<"Size of pose_col_filter: "<<pose_col_filter.size()<<std::endl;
  std::cout<<"size of sphere_col: "<<sphere_color.size()<<std::endl;
  std::cout<<"resolution: "<<res<<std::endl;
  //saveReachMapsToDataset(pose_col_filter, sphere_color, res);
}*/


bool Hdf5Dataset::saveWorkspaceToMap(const map_generation::WorkSpace &ws)
{
  if(!checkPath(this->path_))
  {
    createPath(this->path_);
  }

  /*for(int i=0;i<ws.WsSpheres.size();++i)
  {
    std::cout<<i+1<<" "<<ws.WsSpheres[i].point.x<<" "<<ws.WsSpheres[i].point.y<<" "<<ws.WsSpheres[i].point.z<<std::endl;

  }*/
  //Creating Multimap to Straight vector<vector<double> > //Can take this function to a new class
  std::vector< std::vector< double > > pose_reach;
  std::vector<std::vector<double> > spheres;
  std::vector<double> ri;
  for(int i=0;i<ws.WsSpheres.size();++i)
  {
    std::vector<double> sphere_vec(3);
    reuleaux::pointToVector(ws.WsSpheres[i].point, sphere_vec);
    spheres.push_back(sphere_vec);
    ri.push_back(ws.WsSpheres[i].ri);
    std::vector< double > pose_and_sphere(10);
    for(int j=0;j<3;++j)
    {
      pose_and_sphere[j] = sphere_vec[j];
    }
    for(int k=0;k<ws.WsSpheres[i].poses.size();++k)
    {
      std::vector<double> pose_vec(7);
      reuleaux::poseToVector(ws.WsSpheres[i].poses[k], pose_vec);
      for(int l=0;l<7;++l)
      {
        pose_and_sphere[3+l] = pose_vec[l];
      }
      pose_reach.push_back(pose_and_sphere);
    }
  }

  std::cout<<"size of pose_reach: "<<pose_reach.size()<<std::endl;
  std::cout<<"size of spheres: "<<spheres.size()<<std::endl;
   std::cout<<"size of ri: "<<ri.size()<<std::endl;

  const char *filepath = this->path_.c_str();
  const char *name = this->filename_.c_str();
  char fullpath[100];
  strcpy(fullpath, filepath);
  strcat(fullpath, name);
  ROS_INFO("Saving map %s", this->filename_.c_str());
  this->file_ = H5Fcreate(fullpath, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  this->group_poses_ = H5Gcreate(this->file_, "/Poses", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  this->group_spheres_ = H5Gcreate(this->file_, "/Spheres", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  ROS_INFO("Saving poses in reachability map");
  const hsize_t ndims = 2;
  const hsize_t ncols = 10;

  int posSize =pose_reach.size();
  int chunk_size;
  int PY = 10;
  if (posSize % 2)
      {
        chunk_size = (posSize / 2) + 1;
      }
  else
      {
        chunk_size = (posSize / 2);
      }
  // Create Dataspace
  hsize_t dims[ndims] = {0, ncols};  // Starting with an empty buffer
  hsize_t max_dims[ndims] = {H5S_UNLIMITED, ncols};  // Creating dataspace
  hid_t file_space = H5Screate_simple(ndims, dims, max_dims);

  // Create Dataset Property list
  hid_t plist = H5Pcreate(H5P_DATASET_CREATE);
  H5Pset_layout(plist, H5D_CHUNKED);
  hsize_t chunk_dims[ndims] = {chunk_size, ncols};
  H5Pset_chunk(plist, ndims, chunk_dims);


  // Create the datset
  this->poses_dataset_ = H5Dcreate(this->group_poses_, "poses_dataset", H5T_NATIVE_FLOAT, file_space, H5P_DEFAULT, plist, H5P_DEFAULT);
  // Closing resources
  H5Pclose(plist);
  H5Sclose(file_space);

  // Creating the first buffer
  hsize_t nlines = chunk_size;
  float *buffer = new float[nlines * ncols];
  float **dset1_data = new float *[nlines];
  for (hsize_t i = 0; i < nlines; ++i)
    {
      dset1_data[i] = &buffer[i * ncols];
    }

  // Data for the first chunk
  for (int i = 0; i < chunk_size; i++)
    {
      for (int j = 0; j < PY; j++)
      {
        dset1_data[i][j] = pose_reach[i][j];
      }
    }
  // Memory dataspace indicating size of the buffer
  dims[0] = chunk_size;
  dims[1] = ncols;
  hid_t mem_space = H5Screate_simple(ndims, dims, NULL);

  // Extending dataset
  dims[0] = chunk_size;
  dims[1] = ncols;
  H5Dset_extent(this->poses_dataset_, dims);

  // Selecting hyperslab on the dataset
  file_space = H5Dget_space(this->poses_dataset_);
  hsize_t start[2] = {0, 0};
  hsize_t count[2] = {chunk_size, ncols};
  H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

  // Writing buffer to the dataset
  H5Dwrite(this->poses_dataset_, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

  // Closing file dataspace
  H5Sclose(file_space);
  // Data for the Second chunk
  for (int i = chunk_size; i < posSize; i++)
    {
      for (int j = 0; j < PY; j++)
      {
        dset1_data[i - chunk_size][j] = pose_reach[i][j];
      }
    }

  // Resizing new memory dataspace indicating new size of the buffer
  dims[0] = posSize - chunk_size;
  dims[1] = ncols;
  H5Sset_extent_simple(mem_space, ndims, dims, NULL);

  // Extend dataset
  dims[0] = posSize;
  dims[1] = ncols;
  H5Dset_extent(this->poses_dataset_, dims);
  // Selecting hyperslab
  file_space = H5Dget_space(this->poses_dataset_);
  start[0] = chunk_size;
  start[1] = 0;
  count[0] = posSize - chunk_size;
  count[1] = ncols;
  H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

  // Writing buffer to dataset
  H5Dwrite(this->poses_dataset_, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

  // Closing all the resources
  delete[] dset1_data;
  delete[] buffer;


  // Creating Sphere dataset
  ROS_INFO("Saving spheres in Reachability map");
  hid_t sphere_dataspace;
  const int SX = spheres.size();
  const int SY = 4;

  hsize_t dims2[2];  // dataset dimensions
  dims2[0] = SX;
  dims2[1] = SY;
  double dset2_data[SX][SY];

  for(int i=0;i<spheres.size();++i)
  {
    for(int j=0;j<spheres[i].size();++j)
    {
       dset2_data[i][j] = spheres[i][j];
      //std::cout<<spheres[i][j]<<" ";
    }
    for (int j = 3; j < SY; j++)
    {
      dset2_data[i][j] = 20;//ri[i];
    }

  }


  sphere_dataspace = H5Screate_simple(2, dims2, NULL);
  this->sphere_dataset_ = H5Dcreate2(this->group_spheres_, "sphere_dataset", H5T_NATIVE_DOUBLE,
                                     sphere_dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(this->sphere_dataset_, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, dset2_data);

  // Creating attribute


  hsize_t attr_dims;
  float attr_data[1];
  attr_data[0] = ws.resolution;
  attr_dims = 1;
  sphere_dataspace = H5Screate_simple(1, &attr_dims, NULL);
  this->attr_ = H5Acreate2(this->sphere_dataset_, "Resolution", H5T_NATIVE_FLOAT, sphere_dataspace,
                           H5P_DEFAULT, H5P_DEFAULT);
  H5Awrite(this->attr_, H5T_NATIVE_FLOAT, attr_data);
  //H5Aclose(this->attr_);

  // Closing all

  H5Sclose(sphere_dataspace);
  H5Sclose(file_space);
  H5Sclose(mem_space);
  close();
}

}
