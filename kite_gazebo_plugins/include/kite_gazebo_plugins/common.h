//
// Created by kotaru on 2/10/23.
//

#ifndef KITE_ROS_KITE_GAZEBO_PLUGINS_COMMON_H_
#define KITE_ROS_KITE_GAZEBO_PLUGINS_COMMON_H_

#include <eigen3/Eigen/Dense>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>

#include <functional>
#include <gazebo/gazebo.hh>

#if GAZEBO_MAJOR_VERSION >= 8

#include "gazebo/common/CommonTypes.hh"

using GazeboVector = ignition::math::Vector3d;
using GazeboPose = ignition::math::Pose3d;
using GazeboQuaternion = ignition::math::Quaterniond;

#else // I.E. GAZEBO_MAJOR_VERSION < 8

using GazeboVector = gazebo::math::Vector3;
using GazeboPose = gazebo::math::Pose;
using GazeboQuaternion = gazebo::math::Quaternion;

#endif // GAZEBO_MAJOR_VERSION >= 8

namespace kite_ros {

static Eigen::Vector3d E1{1., 0., 0.};
static Eigen::Vector3d E2{0., 1., 0.};
static Eigen::Vector3d E3{0., 0., 1.};

class RosClock {
public:
  RosClock() {
    now_s = ros::Time::now().toSec();
    last_update_s = ros::Time::now().toSec();
  }
  double now_s{0.};
  double last_update_s{0.};
  double dt_{0.};
  void run() {
    now_s = ros::Time::now().toSec();
    dt_ = (now_s - last_update_s);
    last_update_s = now_s;
  }
  double time() {
    run();
    return dt();
  }
  double dt() const { return dt_; }
};

class GazeboTimer {
public:
  GazeboTimer(const double rate, std::function<void(double)> function)
      : rate_Hz(rate), function(function) {
    req_dt_s = 1. / rate_Hz;
  }

  std::function<void(double)> function;
  double rate_Hz{100.};
  double req_dt_s{0.01};
  gazebo::common::Time last_time_{};

  void set_rate(const double rate) {
    rate_Hz = rate;
    req_dt_s = 1. / rate_Hz;
  }

  void init(const gazebo::common::Time &curr) { last_time_ = curr; }

  void run(const gazebo::common::Time &curr) {
    double curr_dt_s = (curr - last_time_).Double();
    if (rate_Hz > 0 && (curr_dt_s < req_dt_s)) {
      // nothing
    } else {
      function(curr_dt_s);
      last_time_ = curr;
    }
  }
};

inline Eigen::Vector3d vec3GazeboToEigen(GazeboVector vec) {
#if GAZEBO_MAJOR_VERSION >= 8
  return Eigen::Vector3d(vec.X(), vec.Y(), vec.Z());
#else
  return Eigen::Vector(vec.x, vec.y, vec.z);
#endif
}

inline GazeboVector vec3EigenToGazebo(Eigen::Vector3d vec) {
  return GazeboVector(vec(0), vec(1), vec(2));
}

inline Eigen::Quaterniond quaternionGazeboToEigen(GazeboQuaternion quat) {
#if GAZEBO_MAJOR_VERSION >= 8
  return Eigen::Quaterniond(quat.W(), quat.X(), quat.Y(), quat.Z());
#else
  return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
#endif
}

inline Eigen::Matrix3d gazeboQuaternionToEigenMatrix(GazeboQuaternion quat) {
  return quaternionGazeboToEigen(quat).toRotationMatrix();
}

struct Pose3D {
public:
  Pose3D() = default;
  explicit Pose3D(const gazebo::physics::LinkPtr &link_) {
    gpose = link_->WorldCoGPose();
    gvel = link_->RelativeLinearVel();
    gomega = link_->RelativeAngularVel();

#if GAZEBO_MAJOR_VERSION >= 8
    if ((abs(gpose.Pos().X()) > 100) || (abs(gpose.Pos().Y()) > 100) ||
        (abs(gpose.Pos().Z()) > 100)) {
      gzerr << "bad pose reading" << std::endl;
    }
#else
    if ((abs(gpose.Pos().x > 100) || (abs(gpose.Pos().y) > 100) ||
        (abs(gpose.Pos().z) > 100)) {
      gzerr << "bad pose reading" << std::endl;
    }
#endif

    pos = vec3GazeboToEigen(gpose.Pos());
    rot = gazeboQuaternionToEigenMatrix(gpose.Rot());
    vel = vec3GazeboToEigen(gvel);
    omega = vec3GazeboToEigen(gomega);
  }

  GazeboPose gpose{};
  GazeboVector gvel{};
  GazeboVector gomega{};

  Eigen::Vector3d pos{};
  Eigen::Matrix3d rot{};
  Eigen::Vector3d vel{};
  Eigen::Vector3d omega{};
  double t{0};
};

} // namespace kite_ros

#endif // KITE_ROS_KITE_GAZEBO_PLUGINS_COMMON_H_
