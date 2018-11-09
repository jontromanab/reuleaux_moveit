#include <map_generation/discretization.h>


namespace reuleaux
{
Discretization::Discretization()
{
  center_ = octomap::point3d(0,0,0);
  resolution_ = 0.08;
  radius_ = 1.0;
  max_depth_ = 16;
  centers_.resize(0);
  poses_.resize(0);
}

Discretization::Discretization(geometry_msgs::Pose pose, double resolution, double radius ):
  resolution_(resolution), radius_(radius)
{
  center_=octomap::point3d(pose.position.x, pose.position.y, pose.position.z);
  max_depth_ = 16;
  centers_.resize(0);
  poses_.resize(0);
 }

octomap::OcTree* Discretization::generateBoxTree(const octomap::point3d &origin, const double resolution, const double diameter)
{
  octomap::OcTree* tree = new octomap::OcTree(float(resolution)/2);
  for(float x = origin.x() - diameter * 1.5; x<=origin.x() + diameter * 1.5; x+=resolution)
  {
    for(float y = origin.y() - diameter * 1.5; y<=origin.y() + diameter * 1.5; y+=resolution)
    {
      for(float z = 0; z<=origin.z() + diameter * 1.5; z+=resolution)
      {
        octomap::point3d point;
        point.x() = x;
        point.y() = y;
        point.z() = z;
        tree->updateNode(point, true);
      }
    }
  }
  return tree;
}

void Discretization::createCenters(octomap::OcTree *tree, std::vector<geometry_msgs::Point> &centers)
{
  int sphere_count = 0;
  for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth_),end = tree->end_leafs(); it!=end;++it)
    sphere_count++;
  centers.reserve(sphere_count);
  for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth_),end = tree->end_leafs(); it!=end;++it)
  {
    geometry_msgs::Point point;
    point.x = (it.getCoordinate()).x();
    point.y = (it.getCoordinate()).y();
    point.z = (it.getCoordinate()).z();
    centers.push_back(point);
  }
}

int Discretization::getNumOfSpheres()
{
  return centers_.size();
}

void Discretization::createPosesOnSphere(const geometry_msgs::Point &center, const double r, std::vector<geometry_msgs::Pose> &poses)
{
  const double DELTA = M_PI/5.;
  const unsigned MAX_INDEX  (2 * 5 *5);
  static std::vector<geometry_msgs::Point> position_vector(MAX_INDEX);
  static std::vector<tf::Quaternion> quaternion(MAX_INDEX);
  static bool initialized = false;
  if(!initialized)
  {
    initialized = true;
    unsigned index = 0;
    for(double phi = 0; phi<2*M_PI; phi+=DELTA)//Azimuth[0,2PI]
    {
      for(double theta = 0;theta<M_PI;theta +=DELTA) //Elevation[0,2PI]
      {
        position_vector[index].x = cos(phi)*sin(theta);
        position_vector[index].y = sin(phi)*sin(theta);
        position_vector[index].z = cos(theta);
        tf::Quaternion quat;
        quat.setRPY(0, ((M_PI/2)+theta), phi);
        quat.normalize();
        quaternion[index] = quat;
        index++;
      }
    }
  }
  poses.reserve(MAX_INDEX);
  poses.clear();
  geometry_msgs::Pose pose;
  for(int i=0;i<MAX_INDEX;++i)
  {
    pose.position.x = r * position_vector[i].x + center.x;
    pose.position.y = r * position_vector[i].y + center.y;
    pose.position.z = r * position_vector[i].z + center.z;
    pose.orientation.x = quaternion[i].x();
    pose.orientation.y = quaternion[i].y();
    pose.orientation.z = quaternion[i].z();
    pose.orientation.w = quaternion[i].w();
    poses.push_back(pose);
  }
}

void Discretization::createPoses(const std::vector<geometry_msgs::Point> &centers, std::vector<geometry_msgs::Pose> &poses)
{
  poses.reserve(centers.size()*50);
  for(int i=0;i<centers.size();++i)
  {
    map_generation::WsSphere wsSphere;
    wsSphere.point = centers[i];
    static std::vector<geometry_msgs::Pose>  pose;
    createPosesOnSphere(centers[i], resolution_, pose);
    for(int j=0;j<pose.size();++j)
    {
      poses.push_back(pose[j]);
      wsSphere.poses.push_back(pose[j]);
     }
    ws_.WsSpheres.push_back(wsSphere);
  }
}

int Discretization::getNumOfPoses()
{
  return poses_.size();
}

void Discretization::getInitialWorkspace(map_generation::WorkSpace& ws)
{
  ws = ws_;
}

void Discretization::discretize()
{
  octomap::OcTree* tree = generateBoxTree(center_, resolution_, radius_);
  createCenters(tree, centers_);
  createPoses(centers_, poses_);
}

void Discretization::getCenters(std::vector<geometry_msgs::Point> &points)
{
  points = centers_;
}

} //end namespace reuleaux
