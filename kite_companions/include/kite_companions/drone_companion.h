/**
 * @file drone_companion.h
 * @brief Drone companion class ROS interface
 * @author vkotaru
*/
#ifndef KITE_ROS_KITE_COMPANIONS_DRONE_COMPANION_H_
#define KITE_ROS_KITE_COMPANIONS_DRONE_COMPANION_H_

#include <eigen3/Eigen/Dense>
#include <boost/bind.hpp>
#include <mutex>
#include <string>
#include <sys/time.h>
#include <unordered_map>
// #include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace kite_ros {

class DroneCompanion {
private:
  std::string name_;
  std::string ns_;

  ros::NodeHandle *nh_;
    // ddynamic reconfigure
  std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>>
      ddynrec_;

public:
  DroneCompanion(const std::string &name, const std::string &ns="");
  ~DroneCompanion() = default;

  void run();

};

} // namespace kite_ros
#endif // KITE_ROS_KITE_COMPANIONS_DRONE_COMPANION_H_