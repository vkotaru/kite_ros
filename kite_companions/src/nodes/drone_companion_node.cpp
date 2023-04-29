#include "kite_companions/drone_companion.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ROS_INFO("Initializing drone_companion_node");
  // initializing ros node
  ros::init(argc, argv, "drone_companion_node");
  std::string name;
  ros::NodeHandle nh_("~");
  std::string check;
  nh_.getParam("name", name);

  kite_ros::DroneCompanion node_{name};
  node_.Run();
  return 0;
}