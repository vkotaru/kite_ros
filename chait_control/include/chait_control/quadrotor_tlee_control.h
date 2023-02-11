//
// Created by kotaru on 2/11/23.
//

#ifndef CHAIT_ROS_CHAIT_CONTROL_QUADROTOR_TLEE_CONTROL_H_
#define CHAIT_ROS_CHAIT_CONTROL_QUADROTOR_TLEE_CONTROL_H_

#include "chait_control/math.h"
#include <eigen3/Eigen/Dense>

namespace chait_ros {

class SO3Controller {
private:
  Eigen::Vector3d kp_{8.0, 8.0, 3.0};
  Eigen::Vector3d kd_{0.3, 0.3, 0.225};

  Eigen::Matrix3d inertia_{};
  Eigen::Matrix3d inertia_inv_{};
  Eigen::Vector3d moment_{};

  Eigen::Vector3d thrust_vector_{};
  double yaw_sp = 0;

  Eigen::Matrix3d Rd = Eigen::Matrix3d::Identity();
  Eigen::Vector3d Omd = Eigen::Vector3d::Zero();
  Eigen::Vector3d dOmd = Eigen::Vector3d::Zero();

public:
  explicit SO3Controller(const Eigen::Matrix3d &_inertia) {
    inertia_ = _inertia;
    inertia_inv_ = _inertia.inverse();
  }
  SO3Controller()
      : SO3Controller((Eigen::Matrix3d() << 0.0049, 5.5e-06, 5.4e-06, 5.5e-06,
                       0.0053, 2.1e-05, 5.4e-06, 2.1e-05, 0.0098)
                          .finished()) {}
  ~SO3Controller() = default;

  void updateInertia(const Eigen::Matrix3d &inertia) {
    inertia_ = inertia;
    inertia_inv_ = inertia.inverse();
  }

  void updateYawSetpoint(const double &_yaw) { yaw_sp = _yaw; }

  void updateCommand(const Eigen::Vector3d &thrust_v) {
    Eigen::Vector3d E1d{E1}, E1c{E1}, E2c{E2}, E3c{E3};

    thrust_vector_ = thrust_v;
    if (!(std::isnan(thrust_vector_.norm()) ||
          (thrust_vector_.norm() < 1e-4))) {
      E3c = thrust_vector_.normalized();
    }
    E1d = Eigen::Vector3d(std::cos(yaw_sp), std::sin(yaw_sp), 0.0);
    E1c = -hat3d(E3c) * hat3d(E3c) * E1d;
    E2c = E3c.cross(E1c);

    Rd << E1c, E2c, E3c;
  }

  void updateCommand(const Eigen::Vector3d &thrust_v,
                     const Eigen::Vector3d &Omd) {
    updateCommand(thrust_v);
    this->Omd = Omd;
  }
  void updateCommand(const Eigen::Vector3d &thrust_v,
                     const Eigen::Vector3d &Omd, const Eigen::Vector3d &dOmd) {
    updateCommand(thrust_v);
    this->Omd = Omd;
    this->dOmd = dOmd;
  }

  Eigen::Vector3d Update(double dt, Eigen::Matrix3d R, Eigen::Vector3d Om) {
    Eigen::Vector3d eR = 0.5 * vee3d(Rd.transpose() * R - R.transpose() * Rd);
    Eigen::Vector3d eOm = Om - R.transpose() * Rd * Omd;
    Eigen::Vector3d u = -kp_.cwiseProduct(eR) - kd_.cwiseProduct(eOm);
    u += Om.cross(inertia_ * Om);
    u += -inertia_ *
         (Om.cross(R.transpose() * Rd * Omd) - R.transpose() * Rd * dOmd);
    return u;
  }
};
} // namespace chait_ros

#endif // CHAIT_ROS_CHAIT_CONTROL_QUADROTOR_TLEE_CONTROL_H_
