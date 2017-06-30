#include<map_generation/map_generation.h>
#include <sys/types.h>
#include <sys/stat.h>


namespace reuleaux
{
mapGeneration::mapGeneration(ros::NodeHandle& node, const std::string &group_name, const std::string &path,
                             const std::string &filename, const std::string& pkg_name, const double &resolution, const double &radius, bool check_collision)
{
  nh_ = node;
  group_name_ = group_name;
  path_ = path;
  filename_ = filename;
  pkg_name_=pkg_name;
  resolution_ = resolution;
  radius_ = radius;
  check_collision_= check_collision;
  group_.reset(new moveit::planning_interface::MoveGroup(group_name_));
  init_ws_.WsSpheres.clear();
  filtered_ws_.WsSpheres.clear();
}

void mapGeneration::discretizeWorkspace(geometry_msgs::Pose& pose)
{
  reuleaux::Discretization* disc(new reuleaux::Discretization(pose,resolution_, radius_));
  disc->discretize();
  disc->getInitialWorkspace(init_ws_);
  reuleaux::getPoseAndSphereSize(init_ws_, init_sp_size_, init_pose_size_);
  ROS_INFO("Initial workspace has %d spheres and %d poses", init_sp_size_, init_pose_size_);
  delete disc;
}

void mapGeneration::filterWorkspace()
{
  reuleaux::ReachAbility* reach(new reuleaux::ReachAbility(nh_,group_name_, check_collision_));
  reach->setInitialWorkspace(init_ws_);
  reach->createReachableWorkspace();
  reach->getFinalWorkspace(filtered_ws_);
  static int sp_size, pose_size;
  reuleaux::getPoseAndSphereSize(filtered_ws_, final_sp_size_, final_pose_size_);
  delete reach;
}

void mapGeneration::saveWorkspace()
{
  std::string name;
  std::string filename;
  if(filename_ == "default")
  {
    filename = reuleaux::createName(pkg_name_, group_name_, resolution_);
   }
  else
    filename = filename_;
  name = path_+filename;
  reuleaux::Hdf5Dataset* h5(new reuleaux::Hdf5Dataset(name));
  h5->save(filtered_ws_);
  ROS_INFO("%s saved to %s", filename.c_str(), path_.c_str());
 }

void mapGeneration::generate()
{
  ros::Time startit = ros::Time::now();
  getArmPose(arm_pose_);
  discretizeWorkspace(arm_pose_);
  double dif2 = ros::Duration( ros::Time::now() - startit).toSec();
  filterWorkspace();
  double dif3 = ros::Duration( ros::Time::now() - startit).toSec();
  saveWorkspace();
  ROS_INFO("Time for discretizing workspace %.2lf seconds.", dif2);
  ROS_INFO("Center of workspace   x:%f, y:%f, z:%f", arm_pose_.position.x, arm_pose_.position.y, arm_pose_.position.z);
  ROS_INFO("Time for creating reachable workspace is %.2lf seconds.", dif3);
  ROS_INFO("Initial workspace has %d spheres and %d poses", init_sp_size_, init_pose_size_);
  ROS_INFO("Final workspace has %d spheres and %d poses", final_sp_size_, final_pose_size_);
  ROS_INFO("Completed");
}

void mapGeneration::getArmPose(geometry_msgs::Pose& arm_pose)
{
  std::vector<std::string> link_names = group_->getLinkNames();
  std::string first_link = link_names[0];
  moveit::core::RobotModelConstPtr robot_model = group_->getRobotModel();
  geometry_msgs::Pose new_pose;
  new_pose.position.x = 0.0;
  new_pose.position.y = 0.0;
  new_pose.position.z = 0.75;
  new_pose.orientation.w = 1.0;
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  Eigen::Affine3d tf_root_to_first_link = robot_state->getGlobalLinkTransform(first_link);
  tf::poseEigenToMsg(tf_root_to_first_link, arm_pose);
}

}
