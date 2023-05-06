#ifndef KITE_ROS_KITE_COMPANIONS_COMMON_H_
#define KITE_ROS_KITE_COMPANIONS_COMMON_H_

#include "kite_control/kite_control.h"
#include "kite_companions/timer.h"
#include <ros/ros.h>

namespace kite_ros {

struct TrajectoryGenerator {
  /* trajectories */
  trajectory::Setpoint setpoint{};
  trajectory::GoTo go_to{};
  trajectory::Square square{};
  trajectory::VariableFreq var_freq{};
};

struct PositionBuffer {
  PositionBuffer() : x(0.), y(0.), z(0.) {}
  PositionBuffer(const double _x, const double _y, const double _z)
      : x(_x), y(_y), z(_z) {}
  double x, y, z;

  Eigen::Vector3d operator()() {
    return Eigen::Vector3d(x, y, z);
  }

  PositionBuffer operator+(const PositionBuffer &other) {
    return PositionBuffer(x + other.x, y + other.y, z + other.z);
  }
  PositionBuffer operator+=(const PositionBuffer &other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }  
  PositionBuffer operator+(const Eigen::Vector3d &other) {
    return PositionBuffer(x + other[0], y + other[1], z + other[2]);
  }
  PositionBuffer operator+=(const Eigen::Vector3d &other) {
    x += other[0];
    y += other[1];
    z += other[2];
    return *this;
  }
};

static const struct TerminalColors {
  std::string GREEN = "\033[01;32m";
  std::string NC = "\033[0m"; // No Color
  std::string BLACK = "\033[01;30m";
  std::string RED = "\033[01;31m";
  std::string YELLOW = "\033[01;33m";
  std::string BLUE = "\033[01;34m";
  std::string MAGENTA = "\033[01;35m";
  std::string CYAN = "\033[01;36m";
  std::string WHITE = "\033[0;37m";
  std::string Reset = "\033[0m";
} tColors;

struct Logger {

  static void WARN(const std::string &msg) {
    std::string s;
    s += std::string("[WARN] ") + msg;
    std::cout << tColors.YELLOW << s << tColors.NC << std::endl;
  }

  static void ERROR(const std::string &msg) {
    std::string s;
    s += std::string("[ERROR] ") + msg;
    std::cout << tColors.RED + s + tColors.NC << std::endl;
  }
  static void SUCCESS(const std::string &msg) {
    std::string s;
    s += std::string("[SUCCESS] ") + msg;
    std::cout << tColors.GREEN + s + tColors.NC << std::endl;
  }

  static void STATUS(const std::string &msg) {
    std::string s;
    s += std::string("[STATUS] ") + msg;
    std::cout << tColors.CYAN + s + tColors.NC << std::endl;
  }

  static void INFO(const std::string &msg) {
    std::string s;
    s += std::string("[INFO] ") + msg;
    std::cout << tColors.WHITE + s + tColors.NC << std::endl;
  }
};



} // namespace kite_ros
#endif // KITE_ROS_KITE_COMPANIONS_COMMON_H_
