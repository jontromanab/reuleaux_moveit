#ifndef DISCRETIZATION_H
#define DISCRETIZATION_H

#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Point.h>
#include<octomap/octomap.h>
#include<octomap/MapCollection.h>
#include<octomap/math/Utils.h>
#include<map_generation/WorkSpace.h>
#include<tf/LinearMath/Quaternion.h>

namespace  reuleaux
{

class Discretization
{
public:
  Discretization();
  Discretization(geometry_msgs::Pose pose, double resolution = 0.1, double radius = 0.8);

  int getNumOfSpheres();
  int getNumOfPoses();
  void getInitialWorkspace(map_generation::WorkSpace& ws);
  void discretize();

  void getCenters(std::vector<geometry_msgs::Point>& points);

private:
  octomap::OcTree* generateBoxTree(const octomap::point3d& origin, const double resolution, const double diameter);
  void createCenters(octomap::OcTree* tree, std::vector<geometry_msgs::Point>& centers);
  void createPoses(const std::vector<geometry_msgs::Point>& centers, std::vector<geometry_msgs::Pose>& poses);
  void createPosesOnSphere(const geometry_msgs::Point& center, const double resolution, std::vector<geometry_msgs::Pose>& poses);

  double resolution_;
  double radius_;
  octomap::point3d center_;
  unsigned char max_depth_;

  std::vector<geometry_msgs::Point> centers_;
  std::vector<geometry_msgs::Pose> poses_;

  map_generation::WorkSpace ws_;






};

}//end namespace reuleaux


#endif // DISCRETIZATION_H
