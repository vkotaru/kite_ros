//
// Created by kotaru on 2/10/23.
//

#include "chait_gazebo_plugins/chait_quadrotor_plugin.h"

namespace chait_ros {

CHAITQuadrotorPlugin::CHAITQuadrotorPlugin()
    : CHAITGazeboPlugin("CHAITQuadrotorPlugin") {}

void CHAITQuadrotorPlugin::Load(gazebo::physics::ModelPtr _model,
                                sdf::ElementPtr _sdf) {
  loadAndReadDefaultParams(_model, _sdf);
  // read params from sdf specific to this plugin
  if (!_sdf->HasElement("outerUpdateRate")) {
    gzdbg << "[" << plugin_name_
          << "] missing <outerUpdateRate>, "
             "defaults to 100.0"
             " (as fast as possible)\n";
    this->outerloop_Hz = 100.0;
  } else {
    this->outerloop_Hz = _sdf->GetElement("outerUpdateRate")->Get<double>();
    gzdbg << "[" << plugin_name_ << "] outerloop update rate " << outerloop_Hz
          << std::endl;
  }

  // Note, all ros messages are published at the same rate
  if (_sdf->HasElement("publishRate")) {
    this->ros_publish_rate_Hz = _sdf->GetElement("publishRate")->Get<double>();
    gzdbg << "[" << plugin_name_ << "] publish rate " << ros_publish_rate_Hz
          << std::endl;
  }

  // read system parameters
  // Inertial setup
  d.inertia << link_->GetInertial()->IXX(), link_->GetInertial()->IXY(),
      link_->GetInertial()->IXZ(), link_->GetInertial()->IXY(),
      link_->GetInertial()->IYY(), link_->GetInertial()->IYZ(),
      link_->GetInertial()->IXZ(), link_->GetInertial()->IYZ(),
      link_->GetInertial()->IZZ();
  d.inertia_inv = d.inertia.inverse();
  d.mass = link_->GetInertial()->Mass();
  attitude_controller_.updateInertia(d.inertia);

  pub_odom_truth_ = this->nh_->advertise<nav_msgs::Odometry>(
      "chait_quadrotor_plugin/odometry", 1);
  sub_command_ =
      this->nh_->subscribe("chait_quadrotor_plugin/command", 10,
                           &CHAITQuadrotorPlugin::commandCallback, this);

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
  this->outerloop_last_time_ = this->world_->SimTime();
  this->last_ros_publish_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
  this->outerloop_last_time_ = this->world_->GetSimTime();
  this->last_ros_publish_time_ = this->world_->GetSimTime();
#endif

  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&CHAITQuadrotorPlugin::OnUpdate, this, _1));

  gzmsg << "[" << plugin_name_ << "] ... plugin loaded!";
}

void CHAITQuadrotorPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info) {

#if GAZEBO_MAJOR_VERSION >= 8
  cur_time = this->world_->SimTime();
#else
  cur_time = this->world_->GetSimTime();
#endif
  if (cur_time < last_time_) {
    gzdbg << "[QuadrotorSIL] Negative update time difference detected.\n";
    last_time_ = cur_time;
  }

  // outer loop
  if (outerloop_Hz > 0 &&
      (cur_time - outerloop_last_time_).Double() < (1.0 / this->outerloop_Hz)) {
    // too fast; 1./outerloop_Hz time hasn't elapsed yet!
  } else {
    double dt_s = cur_time.Double() - outerloop_last_time_.Double();
    queryState();
    positionControlLoop(dt_s);
    outerloop_last_time_ = cur_time;
  }

  // inner loop
  if (update_rate_Hz > 0 &&
      (cur_time - last_time_).Double() < (1.0 / this->update_rate_Hz)) {
    // too fast; (1./update_rate_Hz) time hasn't elapsed yet!
  } else {
    // compute "dt"
    double dt_s = cur_time.Double() - last_time_.Double();
    queryState();
    attitudeControlLoop(dt_s);
    last_time_ = cur_time;
  }

  // publish
  if (ros_publish_rate_Hz > 0 && (cur_time - last_ros_publish_time_).Double() <
                                     (1.0 / this->ros_publish_rate_Hz)) {
    // too fast; (1./ros_publish_rate_Hz) time hasn't elapsed yet!
  } else {
    rosPublish();
    last_ros_publish_time_ = cur_time;
  }

  // apply wrench to the quadrotor rigid body
  Eigen::Vector3d body_force_body_frame = d.scalar_thrust * E3;
  Eigen::Vector3d body_moment_body_frame = d.moment;
  applyWrench(body_force_body_frame, body_moment_body_frame);

  // reset flags
  is_state_queried_recently = false;
}

void CHAITQuadrotorPlugin::queryState() {
  // TODO use an independent rate for state query
  if (!is_state_queried_recently) {
    auto gz_pose = link_->WorldCoGPose();
    auto gz_vel = link_->RelativeLinearVel();
    auto gz_omega = link_->RelativeAngularVel();

    d.state.position = vec3GazeboToEigen(gz_pose.Pos());
    d.state.velocity = vec3GazeboToEigen(gz_vel);
    d.state.ang_vel = vec3GazeboToEigen(gz_omega);
    d.state.rotation = gazeboQuaternionToEigenMatrix(gz_pose.Rot());

    is_state_queried_recently = true;
  }
}

void CHAITQuadrotorPlugin::positionControlLoop(const double dt) {
  if (mode_ == chait_msgs::QCommand::MODE_POSITION ||
      mode_ == chait_msgs::QCommand::MODE_POSITION_SPLINE) {

    d.thrust_v = d.mass * position_controller_.Update(dt, d.state.position,
                                                      d.state.velocity);
  }
}

void CHAITQuadrotorPlugin::attitudeControlLoop(const double dt) {

  if (mode_ != chait_msgs::QCommand::MODE_PASS_THROUGH) {

    d.scalar_thrust = d.thrust_v.dot(d.state.rotation.col(2));
    attitude_controller_.updateCommand(d.thrust_v);
    d.moment =
        attitude_controller_.Update(dt, d.state.rotation, d.state.ang_vel);
  }
}

void CHAITQuadrotorPlugin::applyWrench(const Eigen::Vector3d &thrust_v,
                                       const Eigen::Vector3d &moment_v) {

  GazeboVector force = vec3EigenToGazebo(thrust_v);
  GazeboVector torque = vec3EigenToGazebo(moment_v);
  link_->AddRelativeForce(force);
  // TODO verify the negative sign
  link_->AddRelativeTorque(torque - link_->GetInertial()->CoG().Cross(force));
}

void CHAITQuadrotorPlugin::rosPublish() {
  auto pose = Pose3D(link_);
  nav_msgs::Odometry odometry_;
  odometry_.header.stamp.sec = world_->SimTime().sec;
  odometry_.header.stamp.nsec = world_->SimTime().nsec;
  odometry_.header.frame_id = "world";
  odometry_.child_frame_id = namespace_ + "/base_link";
  tf::quaternionEigenToMsg(quaternionGazeboToEigen(pose.gpose.Rot()),
                           odometry_.pose.pose.orientation);
  tf::pointEigenToMsg(pose.pos, odometry_.pose.pose.position);
  tf::vectorEigenToMsg(pose.vel, odometry_.twist.twist.linear);
  tf::vectorEigenToMsg(pose.omega, odometry_.twist.twist.angular);
    pub_odom_truth_.publish(odometry_);
}

void CHAITQuadrotorPlugin::commandCallback(
    const chait_msgs::QCommandStamped::ConstPtr &msg) {

  mode_ = msg->command.mode;

  switch (msg->command.mode) {
  case chait_msgs::QCommand::MODE_POSITION: {
    switch (msg->command.values.size()) {
    case 3:
      position_controller_.updateSetpoint(
          Eigen::Vector3d(msg->command.values[0].x, msg->command.values[0].y,
                          msg->command.values[0].z),
          Eigen::Vector3d(msg->command.values[1].x, msg->command.values[1].y,
                          msg->command.values[1].z),
          Eigen::Vector3d(msg->command.values[2].x, msg->command.values[2].y,
                          msg->command.values[2].z));
      break;

    case 2:
      position_controller_.updateSetpoint(
          Eigen::Vector3d(msg->command.values[0].x, msg->command.values[0].y,
                          msg->command.values[0].z),
          Eigen::Vector3d(msg->command.values[1].x, msg->command.values[1].y,
                          msg->command.values[1].z));
      break;

    case 1:
      position_controller_.updateSetpoint(
          Eigen::Vector3d(msg->command.values[0].x, msg->command.values[0].y,
                          msg->command.values[0].z));
      break;

    default:
      break;
    }
  } break;

  case chait_msgs::QCommand::MODE_THRUST_YAW: {
    d.thrust_v =
        Eigen::Vector3d(msg->command.values[0].x, msg->command.values[0].y,
                        msg->command.values[0].z);
    attitude_controller_.updateYawSetpoint(msg->command.yaw[0]);
    break;
  }

  default:
    ROS_WARN("mode: %d is not implement at this time", msg->command.mode);
    break;
  }
}

GZ_REGISTER_MODEL_PLUGIN(CHAITQuadrotorPlugin);

} // namespace chait_ros