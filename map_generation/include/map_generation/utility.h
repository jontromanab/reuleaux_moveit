#ifndef UTILITY_H
#define UTILITY_H


#include<geometry_msgs/Point.h>
#include<geometry_msgs/Pose.h>
#include<map_generation/WorkSpace.h>
#include<iostream>

namespace reuleaux
{
typedef std::vector<double> VecDouble;
typedef std::vector<VecDouble > VecVecDouble;
typedef std::multimap< VecDouble, VecDouble > MultiMap;
typedef std::map<VecDouble, double> MapVecDouble;
 void pointToVector(const geometry_msgs::Point& point, std::vector<double>& data);
 std::vector<double> pointToVector(const geometry_msgs::Point& point);
 geometry_msgs::Point vectorToPoint(const std::vector<double>& data);
 void poseToVector(const geometry_msgs::Pose& pose, std::vector<double>& data);
 geometry_msgs::Pose vectorToPose(const std::vector<double>& data);
 void getPoseAndSphereSize(const map_generation::WorkSpace& ws, int &sphere_size, int &pose_size);
 std::string getRobotName(const std::string pkg_name);
 std::string createName(const std::string& pkg_name, const std::string& group_name, double& res);



}




#endif // UTILITY_H
