#ifndef CHAIT_ROS_CHAIT_CONTROL_TRAJECTORY_H_
#define CHAIT_ROS_CHAIT_CONTROL_TRAJECTORY_H_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "chait_control/data_types.h"

namespace chait_ros::trajectory {
class Trajectory {
public:
  Trajectory(const Point_t &p, size_t d = 6) : point(p), d(d) {
    this->flats_.clear();
  }
  Trajectory() : Trajectory(Eigen::Vector3d::Zero()) {}
  ~Trajectory() = default;

  Point_t point;
  FlatVariable_t flats_;

  size_t d{2};
  virtual FlatVariable_t run(const double t) { return this->flats_; }

  void reset() { this->flats_.clear(); }
};

class Setpoint : public Trajectory {
public:
  Setpoint(const Point_t &p, size_t d = 6) : Trajectory(p, d) {
    this->flats_.clear();
    this->flats_.update_x(p);
  }

  Setpoint() : Setpoint(Eigen::Vector3d::Zero()) {}

  virtual FlatVariable_t run(const double t) override { return this->flats_; }

  void reset(const Point_t &p) {
    this->flats_.clear();
    this->flats_.update_x(p);
  }
};

class GoTo : public Trajectory {
public:
  Point_t start_pt{0., 0., 0.};
  Point_t stop_pt{0., 0., 0};
  double end_time_{4.};
  double half_time{2.};
  double start_time_, t{0};

  Point_t pos_offset{0., 0., 0.};
  Point_t pos_amp{0., 0., 0.};
  Point_t vel_amp{0., 0., 0.};
  Point_t acc_amp{0., 0., 0.};
  Point_t dacc_amp{0., 0., 0.};
  Point_t d2acc_amp{0., 0., 0.};

  void compute_params() {
    pos_offset = 0.5 * (start_pt + stop_pt);
    pos_amp = 0.5 * (stop_pt - start_pt);
    vel_amp = 0.5 * (stop_pt - start_pt) * M_PI / end_time_;
    acc_amp =
        -0.5 * (stop_pt - start_pt) * M_PI * M_PI / (end_time_ * end_time_);
    dacc_amp = -0.5 * (stop_pt - start_pt) * M_PI * M_PI * M_PI /
               (end_time_ * end_time_ * end_time_);
    d2acc_amp = 0.5 * (stop_pt - start_pt) * M_PI * M_PI * M_PI * M_PI /
                (end_time_ * end_time_ * end_time_ * end_time_);
    half_time = 0.5 * end_time_;
  }

public:
  GoTo(const double start_time, const Point_t &start_sp, const Point_t &stop_sp,
       const double t = 4, size_t d = 6)
      : Trajectory(start_sp, d), start_time_(start_time), start_pt(start_sp),
        stop_pt(stop_sp), end_time_(t) {

    compute_params();

    this->flats_.clear();
    this->flats_.update_x(this->start_pt);
  }

  GoTo() : GoTo(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) {}

  FlatVariable_t run(const double time) {
    t = time - start_time_;

    if (t < end_time_) {
      this->flats_.update_x(pos_offset +
                            pos_amp * std::sin(t * M_PI / end_time_ - M_PI_2));
      this->flats_.update_dx(vel_amp * std::cos(t * M_PI / end_time_ - M_PI_2));
      this->flats_.update_d2x(acc_amp *
                              std::sin(t * M_PI / end_time_ - M_PI_2));
      this->flats_.update_d3x(dacc_amp *
                              std::cos(t * M_PI / end_time_ - M_PI_2));
      this->flats_.update_d4x(d2acc_amp *
                              std::sin(t * M_PI / end_time_ - M_PI_2));
    } else {
      this->flats_.clear();
      this->flats_.update_x(stop_pt);
    }

    return this->flats_;
  }

  void reset(const double start_time, const Point_t &start_sp,
             const Point_t &stop_sp, const double t = 4) {

    this->start_pt = start_sp;
    this->stop_pt = stop_sp;
    this->start_time_ = start_time;
    this->end_time_ = t;

    compute_params();
    this->flats_.clear();
    this->flats_.update_x(start_sp);
  }

  // bool done() { return (t > time_); }
};

class Square : public Trajectory {
public:
  double start_time_;
  Point_t center_;
  double side_{1.};
  double time_{2.};

  std::vector<Point_t> corners_;
  std::vector<GoTo> sides_;

  void compute_params() {

    corners_[0] = Point_t(center_(0) + side_, center_(1) - side_, center_(2));
    corners_[1] = Point_t(center_(0) - side_, center_(1) - side_, center_(2));
    corners_[2] = Point_t(center_(0) - side_, center_(1) + side_, center_(2));
    corners_[3] = Point_t(center_(0) + side_, center_(1) + side_, center_(2));
    corners_[4] = Point_t(center_(0) + side_, center_(1) - side_, center_(2));

    for (const auto &c : corners_) {
      std::cout << c.transpose() << std::endl;
    }

    sides_[0].reset(start_time_, center_, corners_.at(0), time_);
    int it = 0;
    for (it = 0; it < 4; ++it) {
      sides_[it + 1].reset(start_time_ + time_ * (it + 1), corners_.at(it),
                           corners_.at(it + 1), time_);
    }
    sides_[it + 1].reset(start_time_ + time_ * (it + 1), corners_.at(4),
                         center_, time_);
  }

public:
  Square(const double start_time, const Point_t &p,
         const double s /* half side*/, const double t, size_t d = 6)
      : Trajectory(p, d), start_time_(start_time), side_(s), center_(p),
        time_(t) {
    this->flats_.clear();
    this->flats_.update_x(center_);

    corners_.clear();
    for (int i = 0; i < 5; ++i) {
      corners_.push_back(Eigen::Vector3d::Zero());
    }
    sides_.clear();
    for (int i = 0; i < 6; ++i) {
      sides_.push_back(GoTo());
    }

    compute_params();
  }

  Square() : Square(0, Eigen::Vector3d::Zero(), 1, 4) {}

  void reset(const double start_time, const Point_t &p,
             const double s /* half side*/, const double t) {
    this->flats_.clear();
    this->flats_.update_x(center_);

    this->start_time_ = start_time;
    this->center_ = p;
    this->side_ = s;
    this->time_ = t;
    compute_params();
  }

  FlatVariable_t run(const double time) override {

    double t = (time - start_time_);

    // std::cout << "t " << t << std::endl;

    if (0 < t && t <= time_) {
      // reach the corner
      // double f = (t) / time_;
      // this->flats_.at(0) = center_ + (corners_.at(0) - center_) * f;
      // this->flats_.at(1) = (corners_.at(0) - center_) * (1 / time_);
      // std::cout << "going to corner 0" << std::endl;
      this->flats_ = sides_.at(0).run(time);

    } else if ((time_ <= t) && t <= 2 * time_) {
      // first side
      // double f = (t - time_) / time_;
      // this->flats_.at(0) =
      //     corners_.at(0) + (corners_.at(1) - corners_.at(0)) * f;
      // this->flats_.at(1) = (corners_.at(1) - corners_.at(0)) * (1 / time_);
      // std::cout << this->flats_.at(0).transpose() << std::endl;
      // std::cout << "going to corner 1" << std::endl;
      this->flats_ = sides_.at(1).run(time);

    } else if ((2 * time_ <= t) && (t <= 3 * time_)) {
      // second side
      // double f = (t - 2 * time_) / time_;
      // this->flats_.at(0) =
      //     corners_.at(1) + (corners_.at(2) - corners_.at(1)) * f;
      // this->flats_.at(1) = (corners_.at(2) - corners_.at(1)) * (1 / time_);
      // std::cout << this->flats_.at(0).transpose() << std::endl;
      // std::cout << "going to corner 2" << std::endl;
      this->flats_ = sides_.at(2).run(time);

    } else if ((3 * time_ <= t) && (t <= 4 * time_)) {
      // third side
      // double f = (t - 3 * time_) / time_;
      // this->flats_.at(0) =
      //     corners_.at(2) + (corners_.at(3) - corners_.at(2)) * f;
      // this->flats_.at(1) = (corners_.at(3) - corners_.at(2)) * (1 / time_);
      // std::cout << this->flats_.at(0).transpose() << std::endl;
      // std::cout << "going to corner 3" << std::endl;
      this->flats_ = sides_.at(3).run(time);

    } else if ((4 * time_ <= t) && (t <= 5 * time_)) {
      // forth side
      // double f = (t - 4 * time_) / time_;
      // this->flats_.at(0) =
      //     corners_.at(3) + (corners_.at(4) - corners_.at(3)) * f;
      // this->flats_.at(1) = (corners_.at(4) - corners_.at(3)) * (1 / time_);
      // // std::cout << this->flats_.at(0).transpose() << std::endl;
      //
      // std::cout << "going to corner 0" << std::endl;
      this->flats_ = sides_.at(4).run(time);

    } else if ((5 * time_ <= t) && (t <= 6 * time_)) {
      // forth side
      // double f = (t - 4 * time_) / time_;
      // this->flats_.at(0) =
      //     corners_.at(3) + (corners_.at(4) - corners_.at(3)) * f;
      // this->flats_.at(1) = (corners_.at(4) - corners_.at(3)) * (1 / time_);
      // // std::cout << this->flats_.at(0).transpose() << std::endl;
      //
      // std::cout << "going to center" << std::endl;
      this->flats_ = sides_.at(5).run(time);

    } else {
      // stop
      this->flats_.clear();
      this->flats_.update_x(center_);
      // std::cout << this->flats_.at(0).transpose() << std::endl;
    }

    // std::cout << this->flats_.at(0).transpose() << std::endl;
    return this->flats_;
  }

  bool done(const double t) { return (t - start_time_) > (6 * time_); }
};

class VariableFreq : public Trajectory {
public:
  enum class TYPE {
    Y_AXIS = 0,
    CIRCLE,
    ELLIPSE,
    ELLISPE3,

    TYPE_COUNT
  };

  struct Params {
    // variable frequency params;
    double f = 0.02;
    double r{1};
    double r1{1};
    double r2{1};
    double r3{0.5};
    Eigen::Vector3d center_{0., 0., 0.5};
    double a{13};
    double b{1.0};
    double c{0.1};
    double t0{80.0};
    double phi{0.0};
    double tf{160.};
    int type{0};
  };

  double start_time_;
  Point_t center_;
  double time_{3.};
  Params p{};
  TYPE type_{TYPE::Y_AXIS};
  Point_t prev_x;
  GoTo goto_strt_pt{};

public:
  // std::function compute_traj;
  VariableFreq() : VariableFreq(0, Eigen::Vector3d::Zero()) {}

  VariableFreq(const double start_time, const Point_t &p, size_t d = 6)
      : Trajectory(p, d), start_time_(start_time), center_(p) {
    reset(start_time, p);
  }

  ~VariableFreq() = default;

  void reset(const double start_time, const Point_t &center) {
    this->start_time_ = start_time;
    this->center_ = center;
    this->prev_x = center;
    goto_strt_pt.reset(start_time, center, get_start_pt(), time_);
  }
  bool done(const double t) { return (t - start_time_ >= p.tf + time_); }

  void update_params(Params &params) { this->p = params; }

  FlatVariable_t run(const double time) {
    double t = time - start_time_;

    if (0 < t && t <= time_) {
      this->flats_ = goto_strt_pt.run(time);

    } else if (t < p.tf + time_) {
      if (p.type == 0) {
        linear_y(t - time_);
      } else if (p.type == 1) {
        circle(t - time_);
      }
    } else {
      this->flats_.clear();
      this->flats_.update_x(prev_x);
    }

    return this->flats_;
  }

  Eigen::Vector3d get_start_pt() {
    if (p.type == 0) {
      return center_ +
             Eigen::Vector3d(
                 0.,
                 p.r * sin((p.a * M_PI * 2.0) / (p.b + exp(-p.c * (0 - p.t0)))),
                 0.);
    } else if (p.type == 1) {
      return center_ +
             Eigen::Vector3d(
                 p.r * cos((p.a * M_PI * 2.0) / (p.b + exp(-p.c * (0 - p.t0)))),
                 p.r * sin((p.a * M_PI * 2.0) / (p.b + exp(-p.c * (0 - p.t0)))),
                 0.);
    } else {
      return center_;
    }
  }

  void circle(const double t) {

    double r = p.r;
    double a = p.a;
    double b = p.b;
    double c = p.c;
    double t0 = p.t0;

    Eigen::Vector3d x =
        center_ + Eigen::Vector3d(
                      r * cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))),
                      r * sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))), 0.);
    this->prev_x = x;
    this->flats_.update_x(x);
    this->flats_.update_dx(
        Eigen::Vector3d(a * c * M_PI * r * exp(-c * (t - t0)) *
                            sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) *
                            1.0 / pow(b + exp(-c * (t - t0)), 2.0) * -2.0,
                        a * c * M_PI * r * exp(-c * (t - t0)) *
                            cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) *
                            1.0 / pow(b + exp(-c * (t - t0)), 2.0) * 2.0,
                        0));
    this->flats_.update_d2x(Eigen::Vector3d(
        (a * a) * (c * c) * (M_PI * M_PI) * r * exp(c * (t - t0) * -2.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * -4.0 +
            a * (c * c) * M_PI * r * exp(-c * (t - t0)) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 2.0) * 2.0 -
            a * (c * c) * M_PI * r * exp(c * (t - t0) * -2.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 3.0) * 4.0,
        (a * a) * (c * c) * (M_PI * M_PI) * r * exp(c * (t - t0) * -2.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * -4.0 -
            a * (c * c) * M_PI * r * exp(-c * (t - t0)) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 2.0) * 2.0 +
            a * (c * c) * M_PI * r * exp(c * (t - t0) * -2.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 3.0) * 4.0,
        0));
    this->flats_.update_d3x(Eigen::Vector3d(
        (a * a) * (c * c * c) * (M_PI * M_PI) * r * exp(c * (t - t0) * -2.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 1.2E+1 -
            (a * a) * (c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 5.0) * 2.4E+1 +
            (a * a * a) * (c * c * c) * (M_PI * M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 6.0) * 8.0 -
            a * (c * c * c) * M_PI * r * exp(-c * (t - t0)) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 2.0) * 2.0 +
            a * (c * c * c) * M_PI * r * exp(c * (t - t0) * -2.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 3.0) * 1.2E+1 -
            a * (c * c * c) * M_PI * r * exp(c * (t - t0) * -3.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 1.2E+1,
        (a * a * a) * (c * c * c) * (M_PI * M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 6.0) * -8.0 +
            (a * a) * (c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -2.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 1.2E+1 -
            (a * a) * (c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 5.0) * 2.4E+1 +
            a * (c * c * c) * M_PI * r * exp(-c * (t - t0)) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 2.0) * 2.0 -
            a * (c * c * c) * M_PI * r * exp(c * (t - t0) * -2.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 3.0) * 1.2E+1 +
            a * (c * c * c) * M_PI * r * exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 1.2E+1,
        0));
    this->flats_.update_d4x(Eigen::Vector3d(
        (a * a) * (c * c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -2.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * -2.8E+1 +
            (a * a) * (c * c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 5.0) * 1.44E+2 -
            (a * a) * (c * c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -4.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 6.0) * 1.44E+2 +
            (a * a * a * a) * (c * c * c * c) * (M_PI * M_PI * M_PI * M_PI) *
                r * exp(c * (t - t0) * -4.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 8.0) * 1.6E+1 -
            (a * a * a) * (c * c * c * c) * (M_PI * M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 6.0) * 4.8E+1 +
            (a * a * a) * (c * c * c * c) * (M_PI * M_PI * M_PI) * r *
                exp(c * (t - t0) * -4.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 7.0) * 9.6E+1 +
            a * (c * c * c * c) * M_PI * r * exp(-c * (t - t0)) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 2.0) * 2.0 -
            a * (c * c * c * c) * M_PI * r * exp(c * (t - t0) * -2.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 3.0) * 2.8E+1 +
            a * (c * c * c * c) * M_PI * r * exp(c * (t - t0) * -3.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 7.2E+1 -
            a * (c * c * c * c) * M_PI * r * exp(c * (t - t0) * -4.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 5.0) * 4.8E+1,
        (a * a * a) * (c * c * c * c) * (M_PI * M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 6.0) * 4.8E+1 -
            (a * a * a) * (c * c * c * c) * (M_PI * M_PI * M_PI) * r *
                exp(c * (t - t0) * -4.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 7.0) * 9.6E+1 -
            (a * a) * (c * c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -2.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 2.8E+1 +
            (a * a) * (c * c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 5.0) * 1.44E+2 -
            (a * a) * (c * c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -4.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 6.0) * 1.44E+2 +
            (a * a * a * a) * (c * c * c * c) * (M_PI * M_PI * M_PI * M_PI) *
                r * exp(c * (t - t0) * -4.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 8.0) * 1.6E+1 -
            a * (c * c * c * c) * M_PI * r * exp(-c * (t - t0)) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 2.0) * 2.0 +
            a * (c * c * c * c) * M_PI * r * exp(c * (t - t0) * -2.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 3.0) * 2.8E+1 -
            a * (c * c * c * c) * M_PI * r * exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 7.2E+1 +
            a * (c * c * c * c) * M_PI * r * exp(c * (t - t0) * -4.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 5.0) * 4.8E+1,
        0));
  }

  void linear_y(const double t) {

    double r = p.r;
    double a = p.a;
    double b = p.b;
    double c = p.c;
    double t0 = p.t0;
    Eigen::Vector3d x =
        center_ +
        Eigen::Vector3d(
            0., p.r * sin((p.a * M_PI * 2.0) / (p.b + exp(-p.c * (t - p.t0)))),
            0.);
    this->prev_x = x;
    this->flats_.update_x(x);
    this->flats_.update_dx(Eigen::Vector3d(
        0,
        p.a * p.c * M_PI * p.r * exp(-p.c * (t - p.t0)) *
            cos((p.a * M_PI * 2.0) / (p.b + exp(-p.c * (t - p.t0)))) * 1.0 /
            pow(p.b + exp(-p.c * (t - p.t0)), 2.0) * 2.0,
        0));
    this->flats_.update_d2x(Eigen::Vector3d(
        0,
        (a * a) * (c * c) * (M_PI * M_PI) * r * exp(c * (t - t0) * -2.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * -4.0 -
            a * (c * c) * M_PI * r * exp(-c * (t - t0)) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 2.0) * 2.0 +
            a * (c * c) * M_PI * r * exp(c * (t - t0) * -2.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 3.0) * 4.0,
        0));
    this->flats_.update_d3x(Eigen::Vector3d(
        0,
        (a * a * a) * (c * c * c) * (M_PI * M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 6.0) * -8.0 +
            (a * a) * (c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -2.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 1.2E+1 -
            (a * a) * (c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 5.0) * 2.4E+1 +
            a * (c * c * c) * M_PI * r * exp(-c * (t - t0)) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 2.0) * 2.0 -
            a * (c * c * c) * M_PI * r * exp(c * (t - t0) * -2.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 3.0) * 1.2E+1 +
            a * (c * c * c) * M_PI * r * exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 1.2E+1,
        0));
    this->flats_.update_d4x(Eigen::Vector3d(
        0,
        (a * a * a) * (c * c * c * c) * (M_PI * M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 6.0) * 4.8E+1 -
            (a * a * a) * (c * c * c * c) * (M_PI * M_PI * M_PI) * r *
                exp(c * (t - t0) * -4.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 7.0) * 9.6E+1 -
            (a * a) * (c * c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -2.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 2.8E+1 +
            (a * a) * (c * c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -3.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 5.0) * 1.44E+2 -
            (a * a) * (c * c * c * c) * (M_PI * M_PI) * r *
                exp(c * (t - t0) * -4.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 6.0) * 1.44E+2 +
            (a * a * a * a) * (c * c * c * c) * (M_PI * M_PI * M_PI * M_PI) *
                r * exp(c * (t - t0) * -4.0) *
                sin((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 8.0) * 1.6E+1 -
            a * (c * c * c * c) * M_PI * r * exp(-c * (t - t0)) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 2.0) * 2.0 +
            a * (c * c * c * c) * M_PI * r * exp(c * (t - t0) * -2.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 3.0) * 2.8E+1 -
            a * (c * c * c * c) * M_PI * r * exp(c * (t - t0) * -3.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 4.0) * 7.2E+1 +
            a * (c * c * c * c) * M_PI * r * exp(c * (t - t0) * -4.0) *
                cos((a * M_PI * 2.0) / (b + exp(-c * (t - t0)))) * 1.0 /
                pow(b + exp(-c * (t - t0)), 5.0) * 4.8E+1,
        0));
  }
};

} // namespace chait_ros::trajectory

#endif // CHAIT_ROS_CHAIT_CONTROL_TRAJECTORY_H_