#include "kite_gazebo_plugins/kite_quadrotor_plugin.h"

namespace kite_ros {

KITEQuadrotorPlugin::KITEQuadrotorPlugin(std::string name)
    : KITEGazeboPlugin(std::move(name)) {}

void KITEQuadrotorPlugin::Load(gazebo::physics::ModelPtr _model,
                               sdf::ElementPtr _sdf) {
  loadAndReadDefaultParams(_model, _sdf);
  // read params from sdf specific to this plugin

  // Note, all ros messages are published at the same rate
  if (_sdf->HasElement("publishRate")) {
    this->ros_publish_rate_Hz = _sdf->GetElement("publishRate")->Get<double>();
    gzdbg << "[" << plugin_name_ << "] publishRate " << ros_publish_rate_Hz
          << std::endl;
  }
  if (_sdf->HasElement("attitudeControlRate")) {
    this->att_loop_rate_Hz =
        _sdf->GetElement("attitudeControlRate")->Get<double>();
    gzdbg << "[" << plugin_name_ << "] attitudeControlRate " << att_loop_rate_Hz
          << std::endl;
  }
  if (_sdf->HasElement("positionControlRate")) {
    this->pos_loop_rate_Hz =
        _sdf->GetElement("positionControlRate")->Get<double>();
    gzdbg << "[" << plugin_name_ << "] positionControlRate " << pos_loop_rate_Hz
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
  attitude_controller_.UpdateInertia(d.inertia);

  pub_odom_truth_ = this->nh_->advertise<nav_msgs::Odometry>(
      "kite_quadrotor_plugin/odometry", 10);
  sub_command_ =
      this->nh_->subscribe("kite_quadrotor_plugin/command", 10,
                           &KITEQuadrotorPlugin::CommandCallback, this);

  this->last_time_ = this->world_->SimTime();

  // initialize timers
  timers_.push_back(std::make_unique<GazeboTimer>(
      pos_loop_rate_Hz, [this](double dt) { this->PositionControlLoop(dt); }));
  timers_.push_back(std::make_unique<GazeboTimer>(
      att_loop_rate_Hz, [this](double dt) { this->AttitudeControlLoop(dt); }));
  timers_.push_back(std::make_unique<GazeboTimer>(
      ros_publish_rate_Hz, [this](double dt) { this->PublishRosTopics(dt); }));

  for (auto &timer : timers_) {
    timer->init(this->world_->SimTime());
  }

  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&KITEQuadrotorPlugin::OnUpdate, this, _1));

  gzmsg << "[" << plugin_name_ << "] ... plugin loaded!";
}

void KITEQuadrotorPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info) {

  curr_time = this->world_->SimTime();
  if (curr_time < last_time_) {
    gzdbg << "[QuadrotorSIL] Negative update time difference detected.\n";
    last_time_ = curr_time;
  }

  QueryState();
  for (auto &timer : timers_) {
    timer->run(curr_time);
  }

  // apply wrench to the quadrotor rigid body
  Eigen::Vector3d body_force_body_frame = d.scalar_thrust * E3;
  Eigen::Vector3d body_moment_body_frame = d.moment;
  ApplyWrench(body_force_body_frame, body_moment_body_frame);

  last_time_ = curr_time;
}

void KITEQuadrotorPlugin::QueryState() {
  // TODO use an independent rate for state query
  auto gz_pose = link_->WorldCoGPose();
  auto gz_vel = link_->RelativeLinearVel();
  auto gz_omega = link_->RelativeAngularVel();

  d.state.position = vec3GazeboToEigen(gz_pose.Pos());
  d.state.velocity = vec3GazeboToEigen(gz_vel);
  d.state.ang_vel = vec3GazeboToEigen(gz_omega);
  d.state.rotation = gazeboQuaternionToEigenMatrix(gz_pose.Rot());
}

void KITEQuadrotorPlugin::PositionControlLoop(const double dt) {

  if (mode_ == kite_msgs::QCommand::MODE_POSITION ||
      mode_ == kite_msgs::QCommand::MODE_POSITION_SPLINE) {

    d.thrust_v = d.mass * position_controller_.Update(dt, d.state.position,
                                                      d.state.velocity);
  }
}

void KITEQuadrotorPlugin::AttitudeControlLoop(const double dt) {

  if (mode_ != kite_msgs::QCommand::MODE_PASS_THROUGH) {
    d.scalar_thrust = d.thrust_v.dot(d.state.rotation.col(2));
    attitude_controller_.UpdateCommand(d.thrust_v);
    d.moment =
        attitude_controller_.Update(dt, d.state.rotation, d.state.ang_vel);
  }
}

void KITEQuadrotorPlugin::ApplyWrench(const Eigen::Vector3d &thrust_v,
                                      const Eigen::Vector3d &moment_v) {

  GazeboVector force = vec3EigenToGazebo(thrust_v);
  GazeboVector torque = vec3EigenToGazebo(moment_v);
  link_->AddRelativeForce(force);
  // TODO verify the negative sign
  link_->AddRelativeTorque(torque - link_->GetInertial()->CoG().Cross(force));
}

void KITEQuadrotorPlugin::PublishRosTopics(const double dt) {
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

void KITEQuadrotorPlugin::CommandCallback(
    const kite_msgs::QCommandStamped::ConstPtr &msg) {

  mode_ = msg->command.mode;

  switch (msg->command.mode) {
  case kite_msgs::QCommand::MODE_POSITION: {
    switch (msg->command.values.size()) {
    case 3:
      position_controller_.UpdateSetpoint(
          Eigen::Vector3d(msg->command.values[0].x, msg->command.values[0].y,
                          msg->command.values[0].z),
          Eigen::Vector3d(msg->command.values[1].x, msg->command.values[1].y,
                          msg->command.values[1].z),
          Eigen::Vector3d(msg->command.values[2].x, msg->command.values[2].y,
                          msg->command.values[2].z));
      break;

    case 2:
      position_controller_.UpdateSetpoint(
          Eigen::Vector3d(msg->command.values[0].x, msg->command.values[0].y,
                          msg->command.values[0].z),
          Eigen::Vector3d(msg->command.values[1].x, msg->command.values[1].y,
                          msg->command.values[1].z));
      break;

    case 1:
      position_controller_.UpdateSetpoint(
          Eigen::Vector3d(msg->command.values[0].x, msg->command.values[0].y,
                          msg->command.values[0].z));
      break;

    default:
      break;
    }
  } break;

  case kite_msgs::QCommand::MODE_THRUST_YAW: {
    if (msg->command.values.size() > 0) {
      d.thrust_v =
          Eigen::Vector3d(msg->command.values[0].x, msg->command.values[0].y,
                          msg->command.values[0].z);
    }
    if (msg->command.yaw.size() > 0) {
      attitude_controller_.UpdateYawSetpoint(msg->command.yaw[0]);
    }
    break;
  }

  default:
    ROS_WARN("mode: %d is not implement at this time", msg->command.mode);
    break;
  }
}

GZ_REGISTER_MODEL_PLUGIN(KITEQuadrotorPlugin);

} // namespace kite_ros