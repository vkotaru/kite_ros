//
// Created by kotaru on 2/25/23.
//

#ifndef SRC_KITE_ROS_KITE_COMPANIONS_COMMON_H_
#define SRC_KITE_ROS_KITE_COMPANIONS_COMMON_H_

#include "kite_control/kite_control.h"

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

#endif // SRC_KITE_ROS_KITE_COMPANIONS_COMMON_H_
