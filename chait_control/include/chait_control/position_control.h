//
// Created by kotaru on 2/11/23.
//
#ifndef CHAIT_ROS_CHAIT_CONTROL_POSITION_CONTROL_H_
#define CHAIT_ROS_CHAIT_CONTROL_POSITION_CONTROL_H_

#include <eigen3/Eigen/Dense>

namespace chait_ros::control {
class PositionController {
private:
  Eigen::Vector3d kp_{4.0, 4.0, 8.0};
  Eigen::Vector3d kd_{3.0, 3.0, 6.0};
  Eigen::Vector3d ki_{0.5, 0.5, 0.5};

  Eigen::Vector3d pos_integral_err{0., 0., 0.};
  bool INTEGRAL_UPDATE = false;
  Eigen::Vector3d ACCEL_UPPER_BOUND{}, ACCEL_LOWER_BOUND{};
  Eigen::Vector3d POS_INTEGRAL_LB{}, POS_INTEGRAL_UB{};

  Eigen::Vector3d xd{0., 0., 0.};
  Eigen::Vector3d vd{0., 0., 0.};
  Eigen::Vector3d ad{0., 0., 0.};

  const double g{9.81};
  const Eigen::Vector3d g_v{0., 0., 9.81};

public:
  PositionController() {
    pos_integral_err.setZero();
    POS_INTEGRAL_LB << -5.0, -5.0, -5.0;
    POS_INTEGRAL_UB << 5.0, 5.0, 5.0;
    ACCEL_LOWER_BOUND << -g, -g, -g;
    ACCEL_UPPER_BOUND << g, g, g;
  }
  ~PositionController() = default;

  virtual void init() {}

  void updateSetpoint(const Eigen::Vector3d &xd) {
    this->xd = xd;
    vd.setZero();
    ad.setZero();
  }
  void updateSetpoint(const Eigen::Vector3d &xd, const Eigen::Vector3d &vd) {
    this->xd = xd;
    this->vd = vd;
    ad.setZero();
  }
  void updateSetpoint(const Eigen::Vector3d &xd, const Eigen::Vector3d &vd,
                      const Eigen::Vector3d &ad) {
    this->xd = xd;
    this->vd = vd;
    this->ad = ad;
  }

  Eigen::Vector3d Update(const double dt, const Eigen::Vector3d &pos,
                         const Eigen::Vector3d &vel) {
    Eigen::Vector3d pos_err = pos - xd;
    Eigen::Vector3d vel_err = vel - vd;
    Eigen::Vector3d cmd_accel;
    cmd_accel.setZero();

    // feedback: PD input
    cmd_accel = -kp_.cwiseProduct(pos_err) - kd_.cwiseProduct(vel_err);

    // integral update
    if (INTEGRAL_UPDATE) {
      pos_integral_err += pos_err * dt;

      // basic anti-windup (bounding the integral error);
      pos_integral_err = (pos_integral_err.cwiseMax(POS_INTEGRAL_LB))
                             .cwiseMin(POS_INTEGRAL_UB);

      // adding integral force
      cmd_accel += -ki_.cwiseProduct(pos_integral_err);
    } else {
      pos_integral_err.setZero();
    }

    // feed-forward input
    cmd_accel += (ad + g_v);

    // input-bounds
    return (cmd_accel.cwiseMax(ACCEL_LOWER_BOUND)).cwiseMin(ACCEL_UPPER_BOUND);
  }
};

} // namespace chait_ros::control

#endif // CHAIT_ROS_CHAIT_CONTROL_POSITION_CONTROL_H_
