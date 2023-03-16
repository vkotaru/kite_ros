#include "chait_companions/drone_companion.h"

namespace chait_ros {

DroneCompanion::DroneCompanion(const std::string &name, const std::string &ns)
    : name_(name), nh_(nullptr) {

  ns_ = ns.empty() ? name : ns + "/" + name;
  nh_ = new ros::NodeHandle(ns);
  // ddynrec_.push_back(std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(*nh_));
}

} // namespace chait_ros
