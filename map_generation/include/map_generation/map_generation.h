#ifndef MAP_GENERATION_H
#define MAP_GENERATION_H

#include<map_generation/discretization.h>
#include<map_generation/utility.h>
#include<map_generation/discretization.h>
#include<map_generation/reachability.h>
#include<map_generation/hdf5_dataset.h>
#include<map_generation/WorkSpace.h>

#include<moveit/move_group_interface/move_group.h>
#include<moveit/robot_state/robot_state.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

namespace reuleaux
{
class mapGeneration
{
public:
  mapGeneration(ros::NodeHandle &node, const std::string& group_name, const std::string& path,
                const std::string& filename, const std::string& pkg_name, const double& resolution, const double& radius, bool check_collision);

  void generate();


private:
   void discretizeWorkspace(geometry_msgs::Pose& pose);
   void filterWorkspace();
   void saveWorkspace();
   void getArmPose(geometry_msgs::Pose& pose);


   ros::NodeHandle nh_;
   std::string group_name_;
   std::string path_;
   std::string filename_;
   std::string pkg_name_;
   double resolution_;
   double radius_;
   bool check_collision_;

   boost::scoped_ptr<moveit::planning_interface::MoveGroup> group_;
   geometry_msgs::Pose arm_pose_;
   map_generation::WorkSpace init_ws_;
   map_generation::WorkSpace filtered_ws_;

   int init_sp_size_, final_sp_size_;
   int init_pose_size_, final_pose_size_;

};


}

#endif // MAP_GENERATION_H
