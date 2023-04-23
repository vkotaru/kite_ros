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

void DroneManager::init() {
  for (auto &controller : position_controllers_map_)
    controller.second->init();
}

void DroneManager::land(const double t) {
  config_.mode = FLIGHT_MODE::LAND;
  config_.start_time = t;
  config_.end_time = 3;

  Eigen::Vector3d end_pt = d.state.position;
  end_pt(2) = d.home_position(2);
  std::cout << "Landing position " << end_pt.transpose() << std::endl;
  traj_gen_.go_to.reset(t, d.state.position, end_pt, 3);
  setControlType(POSITION_CONTROL::POSITION_PID);
}

void DroneManager::takeoff(const double t) {
  config_.mode = FLIGHT_MODE::TAKEOFF;
  config_.start_time = t;
  config_.end_time = 3;

  Eigen::Vector3d end_pt = d.state.position;
  end_pt(2) = 1.5;
  std::cout << "Takeoff position " << end_pt.transpose() << std::endl;
  traj_gen_.go_to.reset(t, d.state.position, end_pt, 3);
  setControlType(POSITION_CONTROL::POSITION_PID);
}

bool DroneManager::setControlType(const POSITION_CONTROL &control_type) {
  if (position_controllers_map_.find(control_type) ==
      position_controllers_map_.end()) {
    Logger::ERROR("Control type not found");
    return false;
  }
  current_ctrl_type = control_type;
  return true;
}

bool DroneManager::setFlightMode(const double t, const EVENT &event_req) {
  switch (config_.mode) {
  case FLIGHT_MODE::IDLE:
    if (event_req == EVENT::REQUEST_TAKEOFF)
      takeoff(t);
    break;

  case FLIGHT_MODE::TAKEOFF:
    if (event_req == EVENT::REQUEST_LANDING)
      land(t);
    break;

  case FLIGHT_MODE::SETPOINT: {
    if (event_req == EVENT::REQUEST_SETPOINT) {
      d.position_setpoint = d.setpoint_buffer;
      traj_gen_.setpoint.reset(d.setpoint_buffer);
      d.flats = traj_gen_.setpoint.run(0);
    } else if (event_req == EVENT::REQUEST_LANDING) {
      land(t);
    } else if (event_req == EVENT::REQUEST_TRAJECTORY) {
      std::cout << "EVENT::REQUEST_TRAJECTORY not implemented" << std::endl;
      return false;
      // config_.mode = FLIGHT_MODE::TRAJECTORY;
      // Eigen::Vector3d center = d.state.position;
      // center(2) = d.flats.x()(2);
      // if (d.traj_type_ == 0) {
      //   traj_gen_.square.reset(t, center, 1, 4);
      // } else if (d.traj_type_ == 1) {
      //   std::cout << "Circular trajectory not implemented yet!" << std::endl;
      // } else if (d.traj_type_ == 2) {
      //   traj_gen_.var_freq.update_params(d.traj_info_.params_);
      //   traj_gen_.var_freq.reset(t, center);
      // }

    } else if (event_req == EVENT::REQUEST_GOTO) {
      config_.mode = FLIGHT_MODE::GOTO;
      config_.start_time = t;
      config_.end_time = 3;
      traj_gen_.go_to.reset(t, d.state.position, d.setpoint_buffer, 3);
    }
    break;
  }

  case FLIGHT_MODE::LAND: {
    std::cout << "Drone landing! Cannot change the mode";
    break;
  }
  case FLIGHT_MODE::TRAJECTORY:
    if (event_req == EVENT::REQUEST_LANDING)
      land(t);
    break;

  default:
    return false;
    break;
  }

  return true;
}

void DroneManager::updateCommandTraj(const double t) {
  switch (config_.mode) {
  case FLIGHT_MODE::TAKEOFF: {
    if (config_.time(t) > config_.end_time) {
      traj_gen_.setpoint.reset(traj_gen_.go_to.stop_pt);
      d.flats = traj_gen_.setpoint.run(0);
      std::cout << "TAKEOFF complete switching to SETPOINT" << std::endl;
      config_.mode = FLIGHT_MODE::SETPOINT;
    } else {
      d.flats = traj_gen_.go_to.run(t);
      std::cout << d.flats.x().transpose() << std::endl;
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
      // Be in drone control mode for landing
    } else {
      d.flats = traj_gen_.go_to.run(t);
    }
  } break;

  case FLIGHT_MODE::TRAJECTORY:
    // if (d.traj_type_ == 0) {
    //   d.flats = traj_gen_.square.run(t);
    //   if (traj_gen_.square.done(t)) {

    //     traj_gen_.setpoint.reset(traj_gen_.square.center_);
    //     d.flats = traj_gen_.setpoint.run(0);

    //     std::cout << "TRAJECTORY complete switching to SETPOINT @ "
    //               << traj_gen_.square.center_.transpose() << std::endl;
    //     config_.mode = FLIGHT_MODE::SETPOINT;
    //   }
    // } else if (d.traj_type_ == 1) {

    // } else if (d.traj_type_ == 2) {
    //   d.flats = traj_gen_.var_freq.run(t);
    //   if (traj_gen_.var_freq.done(t)) {
    //     traj_gen_.setpoint.reset(traj_gen_.var_freq.prev_x);
    //     d.flats = traj_gen_.setpoint.run(0);
    //     std::cout << "TRAJECTORY complete switching to SETPOINT @ "
    //               << traj_gen_.var_freq.prev_x.transpose() << std::endl;
    //     config_.mode = FLIGHT_MODE::SETPOINT;
    //   }
    // }
    break;

  default:

    break;
  }
}


void DroneManager::run(const double t) {
  // update the command
  updateCommandTraj(t);

  // update the controller setpoint
  // TODO: avoid passing this everytime
  position_controllers_map_[current_ctrl_type]->updateSetpoint(d.flats.x(), d.flats.v(), d.flats.a());

  // send the command
  d.thrust_cmd = position_controllers_map_[current_ctrl_type]->Update(t, d.state.position, d.state.velocity);
}

} // namespace kite_ros
