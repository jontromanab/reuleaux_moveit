#include <ros/ros.h>
#include<map_generation/map_generation.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_generation_node");
  ros::NodeHandle nh_("~");



  std::string group_name_, path_, filename_, pkg_name_;;
  bool check_collision_;
  double resolution_, radius_;
  nh_.getParam("group_name", group_name_);
  nh_.getParam("resolution", resolution_);
  nh_.getParam("radius", radius_);
  nh_.getParam("check_collision", check_collision_);
  nh_.getParam("path", path_);
  nh_.getParam("filename", filename_);
  nh_.getParam("pkg_name", pkg_name_);

  reuleaux::mapGeneration mg(nh_, group_name_, path_, filename_,pkg_name_,
                             resolution_, radius_, check_collision_);
  mg.generate();

  ros::spin();




}
