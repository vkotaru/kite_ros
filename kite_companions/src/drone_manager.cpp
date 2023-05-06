//
// Created by kotaru on 2/25/23.
//
#include "kite_companions/drone_manager.h"

namespace kite_ros {

DroneManager::DroneManager() : DroneManager(new Data()) {}

DroneManager::DroneManager(Data *data_ptr) : d(*data_ptr) {

  // create controller map
  // ---------------------
  position_controllers_map_.insert_or_assign(
      POSITION_CONTROL::POSITION_PID,
      std::make_unique<control::PositionController>());
}

void DroneManager::Initialize() {
  for (auto &controller : position_controllers_map_)
    controller.second->init();
}

void DroneManager::LandInternal(const double t) {
  config_.mode = FLIGHT_MODE::LAND;
  config_.start_time = t;
  config_.end_time = 3;

  Eigen::Vector3d end_pt = d.state.position;
  end_pt(2) = d.home_position(2);
  d.setpoint = end_pt;

  std::cout << "Landing position " << end_pt.transpose() << std::endl;
  traj_gen_.go_to.reset(t, d.state.position, end_pt, 3);
  SetControlType(POSITION_CONTROL::POSITION_PID);
}

void DroneManager::TakeoffInternal(const double t) {
  config_.mode = FLIGHT_MODE::TAKEOFF;
  config_.start_time = t;
  config_.end_time = 3;

  d.takeoff_position = d.state.position + Eigen::Vector3d(0, 0, 1.0);
  d.setpoint = d.takeoff_position;

  Eigen::Vector3d end_pt = d.setpoint;
  std::cout << "Takeoff position " << end_pt.transpose() << std::endl;
  traj_gen_.go_to.reset(t, d.state.position, end_pt, 3);
  SetControlType(POSITION_CONTROL::POSITION_PID);
}

bool DroneManager::SetControlType(const POSITION_CONTROL &control_type) {
  if (position_controllers_map_.find(control_type) ==
      position_controllers_map_.end()) {
    Logger::ERROR("Control type not found");
    return false;
  }
  current_ctrl = control_type;
  return true;
}

bool DroneManager::SetFlightMode(const double t, const EVENT &event_req) {
  switch (config_.mode) {
  case FLIGHT_MODE::IDLE:
    if (event_req == EVENT::REQUEST_TAKEOFF)
      TakeoffInternal(t);
    break;

  case FLIGHT_MODE::TAKEOFF:
    if (event_req == EVENT::REQUEST_LANDING)
      LandInternal(t);
    break;

  case FLIGHT_MODE::SETPOINT: {
    if (event_req == EVENT::REQUEST_SETPOINT) {
      traj_gen_.setpoint.reset(d.setpoint);
      d.flats = traj_gen_.setpoint.run(0);
    } else if (event_req == EVENT::REQUEST_LANDING) {
      LandInternal(t);
    } else if (event_req == EVENT::REQUEST_TRAJECTORY) {
      std::cout << "EVENT::REQUEST_TRAJECTORY not implemented" << std::endl;
      return false;

    } else if (event_req == EVENT::REQUEST_GOTO) {
      config_.mode = FLIGHT_MODE::GOTO;
      config_.start_time = t;
      config_.end_time = 3;
      traj_gen_.go_to.reset(t, d.state.position, d.setpoint, 3);
    }
    break;
  }

  case FLIGHT_MODE::LAND: {
    std::cout << "Drone LandInternaling! Cannot change the mode";
    break;
  }
  case FLIGHT_MODE::TRAJECTORY:
    if (event_req == EVENT::REQUEST_LANDING)
      LandInternal(t);
    break;

  default:
    return false;
    break;
  }

  return true;
}

void DroneManager::UpdateCommandTraj(const double t) {
  switch (config_.mode) {
  case FLIGHT_MODE::TAKEOFF: {
    if (config_.time(t) > config_.end_time) {
      traj_gen_.setpoint.reset(traj_gen_.go_to.stop_pt);
      d.flats = traj_gen_.setpoint.run(0);
      std::cout << "TAKEOFF complete switching to SETPOINT" << std::endl;
      config_.mode = FLIGHT_MODE::SETPOINT;
    } else {
      d.flats = traj_gen_.go_to.run(t);
    }
  } break;

  case FLIGHT_MODE::GOTO: {
    if (config_.time(t) > config_.end_time) {
      traj_gen_.setpoint.reset(traj_gen_.go_to.stop_pt);
      d.flats = traj_gen_.setpoint.run(0);
      std::cout << "Reached setpoint!" << std::endl;
      config_.mode = FLIGHT_MODE::SETPOINT;

    } else {
      d.flats = traj_gen_.go_to.run(t);
    }
  } break;

  case FLIGHT_MODE::SETPOINT:
    // nothing to do here;
    // setpoint flats were arleady generated during mode change to SETPOINT
    break;

  case FLIGHT_MODE::LAND: {
    if (config_.time(t) > config_.end_time) {
      traj_gen_.setpoint.reset(traj_gen_.go_to.stop_pt);
      d.flats = traj_gen_.setpoint.run(0);
      std::cout << "Landing complete switching to IDLE" << std::endl;
      config_.mode = FLIGHT_MODE::IDLE;
      // Be in drone control mode for LandInternaling
    } else {
      d.flats = traj_gen_.go_to.run(t);
    }
  } break;

  default:
    break;
  }
}

void DroneManager::Run() {
  if (config_.mode == FLIGHT_MODE::IDLE) {
    d.thrust_cmd = Eigen::Vector3d::Zero();
    return;
  }

  // update the command
  UpdateCommandTraj(d.time_.t_s);

  // update the controller setpoint
  // TODO: avoid passing this everytime
  position_controllers_map_[current_ctrl]->UpdateSetpoint(
      d.flats.x(), d.flats.v(), d.flats.a());

  // send the command
  d.thrust_cmd = position_controllers_map_[current_ctrl]->Update(
      d.time_.t_s, d.state.position, d.state.velocity);
}

} // namespace kite_ros
