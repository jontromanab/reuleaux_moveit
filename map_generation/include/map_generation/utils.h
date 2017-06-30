#ifndef UTILS_H
#define UTILS_H
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Pose.h>
#include<iostream>


namespace reuleaux
{

 void pointToVector(const geometry_msgs::Point& point, std::vector<double>& data);
 std::vector<double> pointToVector(const geometry_msgs::Point& point);
 geometry_msgs::Point vectorToPoint(const std::vector<double>& data);
 void poseToVector(const geometry_msgs::Pose& pose, std::vector<double>& data);
  geometry_msgs::Pose vectorToPose(const std::vector<double>& data);



}

#endif // UTILS_H
