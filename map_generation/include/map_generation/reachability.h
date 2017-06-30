#ifndef REACHABILITY_H
#define REACHABILITY_H
#include<ros/ros.h>
#include<map_generation/WorkSpace.h>
#include<map_generation/utility.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include<moveit_msgs/GetPositionIK.h>
#include<moveit_msgs/PositionIKRequest.h>
#include<moveit/move_group_interface/move_group.h>

namespace reuleaux
{
class ReachAbility
{
public:
  ReachAbility(ros::NodeHandle& node, std::string group_name, bool check_collision);
  void setInitialWorkspace(const map_generation::WorkSpace& initial_ws);
  void getFinalWorkspace(map_generation::WorkSpace& final_ws);
  bool getIKSolution(const geometry_msgs::Pose& pose, moveit_msgs::RobotState& robot_state);
  bool getIKSolution(const geometry_msgs::Pose &pose, std::vector<double>& joint_solution);
  bool getIKSolutionFromTfBase(const geometry_msgs::Pose& base_pose,
                               const geometry_msgs::Pose& pose, moveit_msgs::RobotState& robot_state);
  bool getIKSolutionFromTfBase(const geometry_msgs::Pose &base_pose,
                               const geometry_msgs::Pose &pose, std::vector<double>& joint_solution);
  bool createReachableWorkspace();


private:
  std::string group_name_;
  bool check_collision_;
  geometry_msgs::PoseStamped makePoseStamped(const geometry_msgs::Pose& pose_in);
  moveit_msgs::PositionIKRequest makeServiceRequest(const geometry_msgs::Pose &pose_in);
  bool ik(const moveit_msgs::PositionIKRequest& req, moveit_msgs::RobotState& robot_state);
  void transformTaskpose(const geometry_msgs::Pose& base_pose, const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out);
  bool createReachability(const map_generation::WorkSpace& ws);


  boost::scoped_ptr<moveit::planning_interface::MoveGroup> group_;
  std::string planning_frame_;
  moveit_msgs::GetPositionIK srv_;

  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  map_generation::WorkSpace init_ws_;
  map_generation::WorkSpace final_ws_;
  int pose_size_;
  int sphere_size_;


};

}//end namespace reuleaux

#endif // REACHABILITY_H
