#include "kite_companions/drone_companion.h"

namespace kite_ros {

DroneCompanion::DroneCompanion(const std::string &name, const std::string &ns)
    : name_(name), nh_(nullptr), agent_(&data_) {

  ns_ = ns.empty() ? name : ns + "/" + name;
  nh_ = std::make_unique<ros::NodeHandle>(ns_);
  ROS_WARN("Starting %s drone companion node!", ns_.c_str());
  // ddynrec_.push_back(std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(*nh_));
  data_.time_.start_s = ros::Time::now().toSec();

  // Ros subscribers
  sub_odom_ = nh_->subscribe("odometry", 1, &DroneCompanion::OdometryCallback,
                             this, ros::TransportHints().unreliable());

  // Ros publishers
  pub_cmd_ = nh_->advertise<kite_msgs::QCommandStamped>("command", 1);
}

void DroneCompanion::Clock() { data_.time_.Update(ros::Time::now().toSec()); }

void DroneCompanion::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  ROS_INFO("callback freq: %f",
           1. / (ros::Time::now().toSec() - data_.state.last_update_s));
  data_.state.last_update_s = ros::Time::now().toSec();
  tf::pointMsgToEigen(msg->pose.pose.position, data_.state.position);
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, data_.state.quaternion);
  tf::vectorMsgToEigen(msg->twist.twist.linear, data_.state.velocity);

  ROS_INFO("%f, %f, %f", data_.state.position.x(), data_.state.position.y(),
           data_.state.position.z());
}

void DroneCompanion::Run() {
  DynamicReconfigureSetup();

  // Initialize the agent
  agent_.Initialize();

  ros::Rate run_ros_loop_at(loop_rate_);
  while (ros::ok()) {
    ros::spinOnce();
    Clock();
    // ROS_INFO("loop freq: %f", 1. / data_.time_.dt_s);

    agent_.Run();

    PublishTopics();
    run_ros_loop_at.sleep();
  }

  ROS_WARN("Shutting down %s drone companion node!", ns_.c_str());
}

void DroneCompanion::PublishTopics() {
  // Yet to be implemented
}

void DroneCompanion::RequestTakeoff() {}

void DroneCompanion::RequestLanding() {}

void DroneCompanion::RequestSetpointUpdate() {

}

void DroneCompanion::DynamicReconfigureSetup() {

  ros::NodeHandle nh_cmds(*nh_, "commands");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_cmds =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_cmds);

  ddr_cmds->registerVariable<bool>(
      "REQUEST_TAKEOFF", &flags_.take_off_request,
      [this](bool new_value) {
        if (new_value) {
          this->RequestTakeoff();
          this->flags_.take_off_request = false;
        }
        ROS_WARN("[%s] Requesting takeoff!", this->ns_.c_str());
      },
      "takeoff request flag");

  ddr_cmds->registerVariable<bool>(
      "REQUEST_LANDING", &flags_.landing_request,
      [this](bool new_value) {
        if (new_value) {
          this->RequestLanding();
          this->flags_.landing_request = false;
        }
        ROS_WARN("[%s] Requesting landing!", this->ns_.c_str());
      },
      "landing request flag");

  ddr_cmds->registerEnumVariable<int>(
      std::string("OFFBOARD_CONTROL_MODE"), &offboard_mode_key_,
      [this](int new_value) {
        ROS_INFO("OFFBOARD_CONTROL_MODE::%d", new_value);
      },
      "select the offboard control mode", enum_offboard_mode_map);

  ddr_cmds->registerEnumVariable<int>(
      std::string("POSITION_CONTROL_TYPE"), &pos_ctrl_type_key_,
      [this](int new_value) {
        ROS_INFO("POSITION_CONTROL_TYPE::%d", new_value);
      },
      "select the offboard control mode", enum_position_ctrl_type);

  ddr_cmds->registerVariable<double>(
      "SETPOINT_X", &setpoint_buffer_.x,
      [this](double new_value) { this->setpoint_buffer_.x = new_value; },
      "setpoint position_x", -10.0, 10.0);
  ddr_cmds->registerVariable<double>(
      "SETPOINT_Y", &setpoint_buffer_.y,
      [this](double new_value) { this->setpoint_buffer_.y = new_value; },
      "setpoint position_x", -10., 10.0);
  ddr_cmds->registerVariable<double>(
      "SETPOINT_Z", &setpoint_buffer_.z,
      [this](double new_value) { this->setpoint_buffer_.z = new_value; },
      "setpoint position_x", 0., 10.0);

  ddr_cmds->registerVariable<bool>(
      "UPDATE_SETPOINT", &flags_.update_setpoint,
      [this](double new_value) {
        if (new_value) {
          RequestSetpointUpdate();
          flags_.update_setpoint = false;
          ROS_INFO("Setpoint %f, %f, %f", this->setpoint_.x, this->setpoint_.y,
                   this->setpoint_.z);
        }
      },
      "update setpoint");
  ddr_cmds->registerVariable<bool>(
      "SEND_OFFBOARD_CONTROL", &flags_.send_offboard_commands,
      [this](bool new_value) {
        this->flags_.send_offboard_commands = new_value;
      },
      "start sending offboard control");
  
  // NOTE: there seems to be a limit on the number of registeredVariables
  // Node might crashes for large number of registervariables
  ddr_cmds->publishServicesTopics();
  ddynrec_.push_back(ddr_cmds);

}

} // namespace kite_ros
