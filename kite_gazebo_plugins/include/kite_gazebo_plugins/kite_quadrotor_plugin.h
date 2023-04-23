//
// Created by kotaru on 2/10/23.
//

#ifndef KITE_ROS_KITE_GAZEBO_PLUGINS_KITE_QUADROTOR_PLUGIN_H_
#define KITE_ROS_KITE_GAZEBO_PLUGINS_KITE_QUADROTOR_PLUGIN_H_

#include "kite_gazebo_plugins/kite_gazebo_plugin.h"
#include <eigen_conversions/eigen_msg.h>

#include <kite_msgs/QCommandStamped.h>
#include <nav_msgs/Odometry.h>

#include "kite_control/kite_control.h"

namespace kite_ros {

class KITEQuadrotorPlugin : public KITEGazeboPlugin {
public:
  struct State {
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d ang_vel;
  };

  struct DataContainer {
    DataContainer() {
      inertia.setIdentity();
      inertia_inv.setIdentity();
    }
    double mass{1.};
    Eigen::Matrix3d inertia{};
    Eigen::Matrix3d inertia_inv{};

    State state;

    double scalar_thrust{0.};
    Eigen::Vector3d thrust_v{0., 0., 0.};
    Eigen::Vector3d moment{0., 0., 0.};
  };

  enum class STATE {
    IDLE,
    TAKEOFF,
    HOLD,
    LAND,
    CRASHED,
    COUNT
  };

  

public:
  KITEQuadrotorPlugin();
  ~KITEQuadrotorPlugin() override = default;

protected:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo &_info);

  gazebo::common::Time outerloop_last_time_;
  gazebo::common::Time last_ros_publish_time_;
  double ros_publish_rate_Hz{100.};

  DataContainer d{};
  uint8_t mode_{kite_msgs::QCommand::MODE_THRUST_YAW};

  GazeboPose initial_pose_{};
  void queryState();

  control::PositionController position_controller_{};
  control::SO3Controller attitude_controller_{};

  void positionControlLoop(double dt);
  void attitudeControlLoop(double dt);

  GazeboTimer pos_timer_{100.,
                         [this](double dt) { this->positionControlLoop(dt); }};
  GazeboTimer att_timer_{500.,
                         [this](double dt) { this->attitudeControlLoop(dt); }};

  ros::Publisher pub_odom_truth_;
  ros::Subscriber sub_command_;

  void commandCallback(const kite_msgs::QCommandStamped::ConstPtr &msg);

  // plugin functions
  void applyWrench(const Eigen::Vector3d &thrust_v,
                   const Eigen::Vector3d &moment_v);
  void rosPublish();
};

} // namespace kite_ros
#endif // KITE_ROS_KITE_GAZEBO_PLUGINS_KITE_QUADROTOR_PLUGIN_H_