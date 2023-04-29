/**
 * @file drone_companion.h
 * @brief Drone companion class ROS interface
 * @author vkotaru
 */
#ifndef KITE_ROS_KITE_COMPANIONS_DRONE_COMPANION_H_
#define KITE_ROS_KITE_COMPANIONS_DRONE_COMPANION_H_

#include <boost/bind.hpp>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <sys/time.h>
#include <unordered_map>

#include "kite_companions/drone_manager.h"

#include <nav_msgs/Odometry.h>
#include <kite_msgs/QCommandStamped.h>

namespace kite_ros {

class DroneCompanion {
public:
  DroneCompanion(const std::string &name, const std::string &ns = "");
  ~DroneCompanion() = default;

  void Run();

protected:
  double loop_rate_{100.};

  DroneManager::Data data_{};
  DroneManager agent_;


  // Publishers
  ros::Publisher pub_cmd_;
  // Subscribers
  ros::Subscriber sub_odom_;
  // Callbacks
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

  void PublishTopics();

private:
  std::string name_;
  std::string ns_;

  std::unique_ptr<ros::NodeHandle> nh_;

  // ddynamic reconfigure
  std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>>
      ddynrec_;


  void Clock();
};

} // namespace kite_ros
#endif // KITE_ROS_KITE_COMPANIONS_DRONE_COMPANION_H_