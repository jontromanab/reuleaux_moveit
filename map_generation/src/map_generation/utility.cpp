#include <map_generation/utility.h>
#include <boost/format.hpp>

namespace reuleaux
{
void pointToVector(const geometry_msgs::Point &point, std::vector<double> &data)
{
  data.resize(3);
  data[0] = (double(point.x));
  data[1] = (double(point.y));
  data[2] = (double(point.z));
}

std::vector<double> pointToVector(const geometry_msgs::Point &point)
{
  std::vector<double> data(3);
  data[0] = (double(point.x));
  data[1] = (double(point.y));
  data[2] = (double(point.z));
  return data;
}

geometry_msgs::Point vectorToPoint(const std::vector<double> &data)
{
  geometry_msgs::Point point;
  point.x = data[0];
  point.y = data[1];
  point.z = data[2];
  return point;
}


void poseToVector(const geometry_msgs::Pose &pose, std::vector<double> &data)
{
  data.resize(7);
  data[0]=(double(pose.position.x));
  data[1]=(double(pose.position.y));
  data[2]=(double(pose.position.z));
  data[3]=(double(pose.orientation.x));
  data[4]=(double(pose.orientation.y));
  data[5]=(double(pose.orientation.z));
  data[6]=(double(pose.orientation.w));
}


geometry_msgs::Pose vectorToPose(const std::vector<double>& data)
{
  geometry_msgs::Pose pose;
  pose.position.x = data[0];
  pose.position.y = data[1];
  pose.position.z = data[2];
  pose.orientation.x = data[3];
  pose.orientation.y = data[4];
  pose.orientation.z = data[5];
  pose.orientation.w = data[6];

}

void getPoseAndSphereSize(const map_generation::WorkSpace &ws, int &sphere_size, int &pose_size)
{
  sphere_size = ws.WsSpheres.size();
  pose_size = 0;
  for(int i=0;i<sphere_size;++i)
  {
    for(int j=0;j<ws.WsSpheres[i].poses.size();++j)
      pose_size++;
  }
}

std::string getRobotName(const std::string pkg_name)
{
  std::string mv("_moveit");
  std::string name;
  std::size_t found = pkg_name.find(mv);
  if (found!=std::string::npos)
      name = pkg_name.substr(0, found);
  return name;
}

std::string createName( const std::string &pkg_name, const std::string &group_name, double &res)
{
  std::string robot_name = getRobotName(pkg_name);
  std::string name = str(boost::format("%s_%s_%d_reachability.h5") %robot_name %group_name % res);
  return name;
}



}
