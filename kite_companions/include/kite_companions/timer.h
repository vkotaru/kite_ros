#ifndef KITE_ROS_KITE_COMPANIONS_TIMER_H_
#define KITE_ROS_KITE_COMPANIONS_TIMER_H_

#include <ros/ros.h>

namespace kite_ros {
class Timer {
public:
  Timer() {}
  Timer(const double start_s) {
    this->start_s = start_s;
    this->now_s = start_s;
    this->last_update_s = start_s;
  }

  double start_s{0.};
  double now_s{0.};
  double last_update_s{0.};
  double t_s{0.};
  double dt_s{0.};

  void Update(const double current_s) {
    now_s = current_s;
    dt_s = (now_s - last_update_s);
    t_s = (now_s - start_s);
    last_update_s = now_s;
  }
};
} // namespace kite_ros
#endif // KITE_ROS_KITE_COMPANIONS_TIMER_H_