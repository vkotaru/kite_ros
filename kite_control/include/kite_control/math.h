//
// Created by kotaru on 2/11/23.
//

#ifndef KITE_ROS_KITE_CONTROL_MATH_H_
#define KITE_ROS_KITE_CONTROL_MATH_H_

#include <eigen3/Eigen/Dense>

namespace kite_ros {
namespace control {
/**
 * @brief vee map
 * @param[in] input matrix
 * @return
 */
inline Eigen::Vector3d vee3d(Eigen::Matrix3d M) {
  return (Eigen::Vector3d() << -M(1, 2), M(0, 2), -M(0, 1)).finished();
}

/**
 * @brief hat map
 * @param[in] input vector
 * @return
 */
inline Eigen::Matrix3d hat3d(Eigen::Vector3d v) {
  return (Eigen::Matrix3d() << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0),
          0.0)
      .finished();
}
} // namespace control
} // namespace kite_ros

#endif // KITE_ROS_KITE_CONTROL_MATH_H_
