#include<map_generation/utils.h>

namespace reuleaux
{
void pointToVector(const geometry_msgs::Point &point, std::vector<double> &data)
{
  data.resize(3);
  data[0] = (double(point.x));
  data[1] = (double(point.x));
  data[2] = (double(point.x));
}

std::vector<double> pointToVector(const geometry_msgs::Point &point)
{
  std::vector<double> data(3);
  data[0] = (double(point.x));
  data[1] = (double(point.x));
  data[2] = (double(point.x));
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
}
