//
// Created by kotaru on 2/10/23.
//

#include <utility>

#include "kite_gazebo_plugins/common.h"
#include "kite_gazebo_plugins/kite_gazebo_plugin.h"

namespace kite_ros {

KITEGazeboPlugin::KITEGazeboPlugin(std::string name)
    : gazebo::ModelPlugin(), plugin_name_(std::move(name)), nh_(nullptr) {}

KITEGazeboPlugin::~KITEGazeboPlugin() {
  updateConnection_.reset();
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void KITEGazeboPlugin::loadAndReadDefaultParams(gazebo::physics::ModelPtr _model,
                                          sdf::ElementPtr _sdf) {
  // load parameters from rosparam server
  if (!ros::isInitialized()) {
    ROS_FATAL("A ROS node for Gazebo has not been "
              "initialized, unable to load "
              "plugin");
    return;
  }
  gzmsg << "[" << plugin_name_ << "] plugin loading... ";

  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();
  model_name_ = model_->GetName();
  gzmsg << "[" << plugin_name_ << "] model_name_: " << model_name_ << std::endl;

  ROS_WARN("Searching for namespace... ");
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[" << plugin_name_ << "] Please specify a namespace.\n";

  nh_ = new ros::NodeHandle(namespace_);
  ROS_WARN("namespace found! %s", namespace_.c_str());
  gzmsg << "[" << plugin_name_ << "] loading parameters from " << namespace_
        << " ns\n";

  // look for link name and get the link pointer with that name
  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else
    gzerr << "[KITEGazeboPlugin] Please specify a linkName\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow("[" << plugin_name_ << "] Couldn't find specified link \""
                << link_name_ << "\".");

  // look update updateRate
  if (!_sdf->HasElement("updateRate")) {
    gzdbg << "[" << plugin_name_
          << "] missing <updateRate>, "
             "defaults to 0.0"
             " (as fast as possible)\n";
    this->update_rate_Hz = 0;
  } else {
    this->update_rate_Hz = _sdf->GetElement("updateRate")->Get<double>();
    gzdbg << "[" << plugin_name_ << "] update rate " << update_rate_Hz
          << std::endl;
  }
}

void KITEGazeboPlugin::Load(gazebo::physics::ModelPtr _model,
                             sdf::ElementPtr _sdf) {

  loadAndReadDefaultParams(_model, _sdf);

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif

  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&KITEGazeboPlugin::OnUpdate, this, _1));

  gzmsg << "[" << plugin_name_ << "] ... plugin loaded!";
}

void KITEGazeboPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info) {

#if GAZEBO_MAJOR_VERSION >= 8
  cur_time = this->world_->SimTime();
#else
  cur_time = this->world_->GetSimTime();
#endif

  if (cur_time < last_time_) {
    gzdbg << "[QuadrotorSIL] Negative update time difference detected.\n";
    last_time_ = cur_time;
  }
  // rate control
  if (update_rate_Hz > 0 &&
      (cur_time - last_time_).Double() < (1.0 / this->update_rate_Hz)) {
    // too fast; 1./update_rate_Hz time hasn't elapsed yet!
  } else {
    // compute "dt"
    loop_dt_s = cur_time.Double() - last_time_.Double();
    printf("loop_dt_s_freq, %f\n", 1. / loop_dt_s);
    run();
    last_time_ = cur_time;
  }
}

void KITEGazeboPlugin::run() {}

GZ_REGISTER_MODEL_PLUGIN(KITEGazeboPlugin);

} // namespace kite_ros
