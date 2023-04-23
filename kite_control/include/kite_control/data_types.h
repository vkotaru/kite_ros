#ifndef KITE_ROS_KITE_CONTROL_DATA_TYPES_H_
#define KITE_ROS_KITE_CONTROL_DATA_TYPES_H_

#include <eigen3/Eigen/Dense>
#include <memory>

namespace kite_ros {

using Point_t = Eigen::Vector3d;
//#define G_SI 9.80665
#define O3 Eigen::Matrix3d::Zero()
#define I3 Eigen::Matrix3d::Identity()
static const Eigen::Vector3d GE3{0., 0., 9.81};
template <size_t T> using vec = Eigen::Matrix<double, T, 1>;

struct RigidbodyState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* constructors */
  RigidbodyState() {
    position = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    accel.setZero();
    angular_velocity = Eigen::Vector3d::Zero();
    quaternion = Eigen::Quaterniond();
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }
  RigidbodyState(const Eigen::Vector3d &_x, const Eigen::Vector3d &_v,
                 const Eigen::Quaterniond &_q, const Eigen::Vector3d &_Om) {
    position = _x;
    velocity = _v;
    quaternion = _q;
    angular_velocity = _Om;
  }
  RigidbodyState(const RigidbodyState &other) {
    position = other.position;
    velocity = other.velocity;
    quaternion = other.quaternion;
    angular_velocity = other.angular_velocity;
    accel = other.accel;
  }

  /* destructor */
  ~RigidbodyState() = default;

  Eigen::Vector3d position, velocity, accel;
  Eigen::Matrix3d rotation() { return quaternion.toRotationMatrix(); }
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d euler;
  Eigen::Quaterniond quaternion;
  double roll{}, pitch{}, yaw{};
};

struct CableState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d q{0., 0., -1};
  Eigen::Vector3d omega{0., 0., 0};
  Eigen::Vector3d dq{0., 0., 0.};
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  float l{1.0};
};

class FlatVariable_t {
  size_t n{3};
  std::vector<Point_t> _x;

public:
  FlatVariable_t(size_t _n = 6) : n{_n} {
    _x = std::vector<Point_t>(n, Eigen::Vector3d::Zero());
  }
  Point_t &operator[](size_t i) { return _x[i]; }
  Point_t at(size_t i) { return _x[i]; }
  Point_t at(size_t i) const { return _x[i]; }

  size_t size() const { return n; }

  void clear() {
    for (auto &p : _x) {
      p.setZero();
    }
  }

  void reset() { _x.clear(); }

  void push_back(const Point_t &p) { _x.push_back(p); }

  void resize(size_t _n) {
    n = _n;
    _x.resize(n);
    clear();
  }

  void update_x(const Point_t &p) { _x[0] = p; }
  void update_dx(const Point_t &p) { _x[1] = p; }
  void update_d2x(const Point_t &p) { _x[2] = p; }
  void update_d3x(const Point_t &p) { _x[3] = p; }
  void update_d4x(const Point_t &p) { _x[4] = p; }
  void update(const Point_t &p, const size_t i) { _x[i] = p; }

  Point_t x() const { return _x[0]; } // position
  Point_t v() const { return _x[1]; } // velocity
  Point_t a() const { return _x[2]; } // acceleration
  Point_t j() const { return _x[3]; } // jerk
  Point_t s() const { return _x[4]; } // snap
};

struct S2Trajectory_t {
  Point_t q{0., 0., -1};
  Point_t omega{0., 0., 0.};
  Point_t domega{0., 0., 0};
  Point_t dq{0., 0., 0};
  Point_t d2q{0., 0., 0};

  Point_t compute_dq() { return omega.cross(q); }
  Point_t compute_d2q() { return (domega.cross(q) - omega.dot(omega) * q); }
};

struct SE3Trajectory_t {
  Point_t p{0., 0., 0};
  Point_t v{0., 0., 0.};
  Point_t a{0., 0., 0.};
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Point_t Om{0., 0., 0};
  Point_t dOm{0., 0., 0.};
};

struct CableLoadFlatsNStates {
  CableLoadFlatsNStates(FlatVariable_t &_flats) : flats(_flats) {}
  FlatVariable_t &flats;

  Point_t q{0., 0., -1};
  Point_t omega{0., 0., 0.};
  Point_t domega{0., 0., 0};
  Point_t dq{0., 0., 0};
  Point_t d2q{0., 0., 0};

  Point_t Tp{0., 0., 0.};
  Point_t dTp{0., 0., 0.};
  Point_t d2Tp{0., 0., 0.};

  Point_t xQ{0., 0., 0.};
  Point_t vQ{0., 0., 0.};
  Point_t aQ{0., 0., 0.};

  Point_t F{0., 0., 0.};
};

} // namespace kite_ros

#endif // KITE_ROS_KITE_CONTROL_DATA_TYPES_H