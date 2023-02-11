//
// Created by kotaru on 2/10/23.
//

#ifndef CHAIT_ROS_CHAIT_GAZEBO_PLUGINS_CHAIT_QUADROTOR_PLUGIN_H_
#define CHAIT_ROS_CHAIT_GAZEBO_PLUGINS_CHAIT_QUADROTOR_PLUGIN_H_

#include "chait_gazebo_plugins/chait_gazebo_plugin.h"
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>
#include <chait_msgs/QCommandStamped.h>

namespace chait_ros {

class CHAITQuadrotorPlugin : public CHAITGazeboPlugin {
public:
  struct DataContainer {
    DataContainer() {
      inertia.setIdentity();
      inertia_inv.setIdentity();
    }
    double mass{1.};
    Eigen::Matrix3d inertia{};
    Eigen::Matrix3d inertia_inv{};

    double scalar_thrust{1.};
    Eigen::Vector3d moment{0., 0., 0.};

  };

public:
  CHAITQuadrotorPlugin();
  ~CHAITQuadrotorPlugin() = default;

protected:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo &_info);

  gazebo::common::Time outerloop_last_time_;
  double outerloop_Hz{100.};
  double outerloop_dt_s{0.01};

  int MAX_POS_CTRL_COUNT = 3;
  int pos_ctrl_counter = 0;

  DataContainer d{};
  uint8_t mode_{chait_msgs::QCommand::MODE_THRUST_YAW};

  GazeboPose initial_pose_{};
  void queryState();
  bool is_state_queried_recently{false};

  void computePosInput();
  void computeAttInput();
  ros::Publisher pub_odom_truth_;
  ros::Subscriber sub_command_;

   void commandCallback(const chait_msgs::QCommandStamped::ConstPtr &msg);

  // plugin functions
  void applyWrench(const Eigen::Vector3d &thrust_v,
                   const Eigen::Vector3d &moment_v);
  void rosPublish();
};

} // namespace chait_ros
#endif // CHAIT_ROS_CHAIT_GAZEBO_PLUGINS_CHAIT_QUADROTOR_PLUGIN_H_
