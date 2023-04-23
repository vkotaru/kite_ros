
#ifndef KITE_ROS_KITE_COMPANIONS_DRONE_MANAGER_H_
#define KITE_ROS_KITE_COMPANIONS_DRONE_MANAGER_H_

#include <eigen3/Eigen/Dense>
#include <map>
#include <memory>
#include <iostream>

#include "kite_companions/common.h"
#include "kite_control/kite_control.h"

namespace kite_ros {

/**
 * Dummy "firmware" for quadrotor
 */
class DroneManager {
public:
  enum class POSITION_CONTROL {
    POSITION_PID = 0,
    POSITION_CLF_QP,
    POSITION_CBF_QP,
    POSITION_CLF_CBF_QP,
    POSITION_MPC,

    POSITION_CONTROL_COUNT
  };

  enum class EVENT {
    REQUEST_IDLE = 0,
    REQUEST_TAKEOFF,
    REQUEST_LANDING,
    REQUEST_TRAJECTORY,
    REQUEST_GOTO,
    REQUEST_SETPOINT,

    REQUEST_COUNT
  };

  enum class FLIGHT_MODE {
    IDLE = 0,
    TAKEOFF,
    LAND,
    SETPOINT,
    GOTO,
    TRAJECTORY,

    FLIGHT_MODE_COUNT
  };

  struct FlightConfig {
    /// mission type
    FLIGHT_MODE mode{FLIGHT_MODE::IDLE};
    /// trajectory start time
    double start_time{0.};
    /// trajectory current time
    double end_time{0.};
    /// mission time
    double time(const double &t) const { return t - start_time; }
  };

  struct Data {
      struct State {
          Eigen::Vector3d position;
          Eigen::Vector3d velocity;
          Eigen::Matrix3d orientation;
          Eigen::Vector3d angular_velocity;
      } state;

      Eigen::Vector3d home_position;
      Eigen::Vector3d setpoint_buffer;
      Eigen::Vector3d position_setpoint;
      FlatVariable_t flats;

      Eigen::Vector3d thrust_cmd;
  };

protected:
  Data &d;
  TrajectoryGenerator traj_gen_{};

  std::map<POSITION_CONTROL, std::unique_ptr<control::PositionController>>
      position_controllers_map_;
  POSITION_CONTROL current_ctrl_type{POSITION_CONTROL::POSITION_PID};

  FlightConfig config_{};

  void land(const double t);
  void takeoff(const double t);

public:
  DroneManager();
  explicit DroneManager(Data *data_ptr);

  ~DroneManager() = default;

  void init();
  void updateCommandTraj(const double t);
  void run(const double t);

  bool setFlightMode(const double t, const EVENT& event_req);
  bool setControlType(const POSITION_CONTROL &control_type);

  // getters
  inline const Data &data() const { return d; }
  inline const FLIGHT_MODE &flight_mode() const { return config_.mode; }

};

} // namespace kite_ros

#endif // KITE_ROS_KITE_COMPANIONS_DRONE_MANAGER_H_