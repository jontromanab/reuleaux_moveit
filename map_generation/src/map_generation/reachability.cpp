#include<map_generation/reachability.h>

namespace reuleaux
{
ReachAbility::ReachAbility(ros::NodeHandle& node, std::string group_name, bool check_collision)
  :group_name_(group_name), check_collision_(check_collision)
{
  nh_ = node;
  client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
  planning_frame_ = group_->getPlanningFrame();
  final_ws_.WsSpheres.clear();
  init_ws_.WsSpheres.clear();

}

 geometry_msgs::PoseStamped ReachAbility::makePoseStamped(const geometry_msgs::Pose& pose_in)
{
  geometry_msgs::PoseStamped pose_st;
  pose_st.header.frame_id = planning_frame_;
  pose_st.pose = pose_in;
  return pose_st;
}

 moveit_msgs::PositionIKRequest ReachAbility::makeServiceRequest(const geometry_msgs::Pose &pose_in)
 {
   moveit_msgs::PositionIKRequest req;
   geometry_msgs::PoseStamped pose_st = makePoseStamped(pose_in);
   req.group_name = group_name_;
   req.avoid_collisions = check_collision_;
   req.attempts = 10;
   req.timeout.fromSec(0.1);
   req.pose_stamped = pose_st;
   return req;
 }

 void ReachAbility::setInitialWorkspace(const map_generation::WorkSpace &initial_ws)
 {
   init_ws_ = initial_ws;
   reuleaux::getPoseAndSphereSize(init_ws_, sphere_size_, pose_size_);
 }

 void ReachAbility::getFinalWorkspace(map_generation::WorkSpace &final_ws)
 {
   final_ws = final_ws_;
 }

 void ReachAbility::transformTaskpose(const geometry_msgs::Pose &base_pose, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out)
 {
   //First get the transform between the new base pose to world (inv)
   Eigen::Affine3d base_pose_tf;
   tf::poseMsgToEigen(base_pose, base_pose_tf);
   Eigen::Affine3d base_pose_to_world_tf = base_pose_tf.inverse();
   //Get the transform between the task pose to world
   Eigen::Affine3d reach_pose_tf;
   tf::poseMsgToEigen(pose_in, reach_pose_tf);
   //transform the task pose to base pose at center. Now we can get Ik for this pose
   tf::poseEigenToMsg(reach_pose_tf * base_pose_to_world_tf, pose_out);
 }

 bool ReachAbility::ik(const moveit_msgs::PositionIKRequest& req, moveit_msgs::RobotState &robot_state)
 {
   srv_.request.ik_request = req;
   if(client_.call(srv_))
   {
     if(srv_.response.error_code.val == 1)
     {
       robot_state = srv_.response.solution;
       return true;
     }
     else
       return false;
    }
   else
   {
     ROS_ERROR("Failed to call IK service");
           return 1;
   }
 }

 bool ReachAbility::getIKSolution(const geometry_msgs::Pose &pose, moveit_msgs::RobotState &robot_state)
 {
   moveit_msgs::PositionIKRequest req = makeServiceRequest(pose);
   if(ik(req, robot_state))
     return true;
   else
     return false;
 }

 bool ReachAbility::getIKSolution(const geometry_msgs::Pose &pose, std::vector<double> &joint_solution)
 {
   moveit_msgs::PositionIKRequest req = makeServiceRequest(pose);
   moveit_msgs::RobotState robot_state;
   std::vector<std::string> joint_names;
   if(ik(req, robot_state))
   {
     std::vector<std::string> full_names = robot_state.joint_state.name;
     joint_names = group_->getJointNames();
     for(int i=0;i<joint_names.size();++i)
     {
       int position = std::find(full_names.begin(), full_names.end(), joint_names[i]) - full_names.begin();
       joint_solution.push_back(robot_state.joint_state.position[position]);
     }
     return true;
   }
   else
     return false;
 }

 bool ReachAbility::getIKSolutionFromTfBase(const geometry_msgs::Pose &base_pose, const geometry_msgs::Pose &pose, moveit_msgs::RobotState &robot_state)
 {
   geometry_msgs::Pose task_pose;
   transformTaskpose(base_pose, pose, task_pose);
   moveit_msgs::PositionIKRequest req = makeServiceRequest(task_pose);
   if(ik(req, robot_state))
     return true;
   else
     return false;
 }

 bool ReachAbility::getIKSolutionFromTfBase(const geometry_msgs::Pose &base_pose, const geometry_msgs::Pose &pose, std::vector<double> &joint_solution)
 {
   geometry_msgs::Pose task_pose;
   transformTaskpose(base_pose, pose, task_pose);
   moveit_msgs::PositionIKRequest req = makeServiceRequest(task_pose);
   moveit_msgs::RobotState robot_state;
   std::vector<std::string> joint_names;
   if(ik(req, robot_state))
   {
     std::vector<std::string> full_names = robot_state.joint_state.name;
     joint_names = group_->getJointNames();
     for(int i=0;i<joint_names.size();++i)
     {
       int position = std::find(full_names.begin(), full_names.end(), joint_names[i]) - full_names.begin();
       joint_solution.push_back(robot_state.joint_state.position[position]);
     }
     return true;
   }
   else
     return false;

 }

 bool ReachAbility::createReachableWorkspace()
 {
   if(createReachability(init_ws_))
     return true;
   else
     return false;
 }

 bool ReachAbility::createReachability(const map_generation::WorkSpace& ws)
 {
   reuleaux::MultiMap ws_map;
   reuleaux::MapVecDouble sp_map;
   int sp_size = sphere_size_;
   for(int i=0;i<sp_size;++i)
   {
     ROS_INFO("Processing sphere: %d / %d", i+1,sp_size);
     std::vector<double> sp_vec;
     reuleaux::pointToVector(ws.WsSpheres[i].point, sp_vec);
     for(int j=0;j<ws.WsSpheres[i].poses.size();++j)
     {
       moveit_msgs::RobotState state;
       geometry_msgs::Pose reach_pose= ws.WsSpheres[i].poses[j];
       bool is_reachable = getIKSolution(reach_pose, state);
       if(is_reachable)
       {
         std::vector<double> sp_pose;
         reuleaux::poseToVector(reach_pose, sp_pose);
         ws_map.insert(std::make_pair(sp_vec, sp_pose));
       }
     }
   }
   for(reuleaux::MultiMap::iterator it=ws_map.begin(); it!=ws_map.end();++it)
   {
     std::vector<double> sp_coord = it->first;
     float d = float(ws_map.count(sp_coord)) / (pose_size_ /sphere_size_) * 100;
     sp_map.insert(std::make_pair(it->first, double(d)));
   }
   for (reuleaux::MapVecDouble::iterator it = sp_map.begin(); it != sp_map.end(); ++it)
   {
     map_generation::WsSphere wss;
     wss.point.x = (it->first)[0];
     wss.point.y = (it->first)[1];
     wss.point.z = (it->first)[2];
     wss.ri = it->second;
     for (MultiMap::iterator it1 = ws_map.lower_bound(it->first); it1 != ws_map.upper_bound(it->first); ++it1)
     {
       geometry_msgs::Pose pp;
       pp.position.x = (it1->second)[0];
       pp.position.y = (it1->second)[1];
       pp.position.z = (it1->second)[2];
       pp.orientation.x = (it1->second)[3];
       pp.orientation.y = (it1->second)[4];
       pp.orientation.z = (it1->second)[5];
       pp.orientation.w = (it1->second)[6];
       wss.poses.push_back(pp);
     }
   final_ws_.WsSpheres.push_back(wss);
   }
   final_ws_.resolution = init_ws_.resolution;
 }

}
