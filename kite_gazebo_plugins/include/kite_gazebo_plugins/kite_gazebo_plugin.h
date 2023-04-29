//
// Created by kotaru on 2/10/23.
//

#ifndef KITE_ROS_KITE_GAZEBO_PLUGINS_KITE_GAZEBO_PLUGIN_H_
#define KITE_ROS_KITE_GAZEBO_PLUGINS_KITE_GAZEBO_PLUGIN_H_

#include "kite_gazebo_plugins/common.h"
#include "kite_gazebo_plugins/kite_gazebo_plugin.h"
#include <iostream>

namespace kite_ros {

class KITEGazeboPlugin : public gazebo::ModelPlugin {
public:
  explicit KITEGazeboPlugin(std::string name = "KITEGazeboPlugin");
  ~KITEGazeboPlugin() override;

protected:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  virtual void OnUpdate(const gazebo::common::UpdateInfo &_info);
  void loadAndReadDefaultParams(gazebo::physics::ModelPtr _model,
                                          sdf::ElementPtr _sdf);
  virtual void run();

  std::string plugin_name_;

  gazebo::common::Time plugin_loaded_time_;
  gazebo::common::Time last_plugin_update_;
  gazebo::common::Time last_time_;
  gazebo::common::Time curr_time;
  double update_rate_Hz{500.};
  double loop_dt_s{0.002};

  std::string namespace_;
  std::string link_name_;
  std::string model_name_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::EntityPtr parent_link_;

  gazebo::event::ConnectionPtr updateConnection_;

  ros::NodeHandle *nh_;
};

} // namespace kite_ros

#endif // KITE_ROS_KITE_GAZEBO_PLUGINS_KITE_GAZEBO_PLUGIN_H_
