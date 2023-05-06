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
#include <kite_msgs/QCommand.h>
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
  void Clock();

  // User interface
  // ddynamic reconfigure
  std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>>
      ddynrec_;

  void DynamicReconfigureSetup();

  std::map<std::string, int> enum_offboard_mode_map = {
      {"MODE_PASS_THROUGH", kite_msgs::QCommand::MODE_PASS_THROUGH},
      {"MODE_ATTITUDE", kite_msgs::QCommand::MODE_ATTITUDE},
      {"MODE_ATTITUDE_RATE", kite_msgs::QCommand::MODE_ATTITUDE_RATE},
      {"MODE_THRUST_YAW", kite_msgs::QCommand::MODE_THRUST_YAW}};
  int offboard_mode_key_{2};

  std::map<std::string, int> enum_position_ctrl_type = {
      {"POSITION_PID", static_cast<int>(DroneManager::POSITION_CONTROL::POSITION_PID)},
      {"POSITION_CLF_QP", static_cast<int>(DroneManager::POSITION_CONTROL::POSITION_CLF_QP)}};
  int pos_ctrl_type_key_{0};
  
  PositionBuffer setpoint_{0., 0., 0.};
  PositionBuffer takeoff_position_{0., 0., 0.};
  PositionBuffer setpoint_buffer_{};
  struct Flags {
    bool update_setpoint{false};
    bool send_offboard_commands{false};
    bool take_off_request{false};
    bool landing_request{false};
    bool square_trajectory{false};
    bool circular_trajectory{false};
    bool trajectory_tracking_request{false};
    bool goto_enabled{false};
  };
  Flags flags_;

  void RequestTakeoff();
  void RequestLanding();
  void RequestSetpointUpdate();

};

} // namespace kite_ros
#endif // KITE_ROS_KITE_COMPANIONS_DRONE_COMPANION_H_