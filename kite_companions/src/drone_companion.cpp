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
  sub_odom_ = nh_->subscribe("odometry", 1, &DroneCompanion::OdometryCallback, this, ros::TransportHints().unreliable());

  // Ros publishers
  pub_cmd_ = nh_->advertise<kite_msgs::QCommandStamped>("command", 1);
}

void DroneCompanion::Clock() {
  data_.time_.Update(ros::Time::now().toSec());
}

void DroneCompanion::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  ROS_INFO("callback freq: %f", 1./(ros::Time::now().toSec() - data_.state.last_update_s));
  data_.state.last_update_s = ros::Time::now().toSec();
  tf::pointMsgToEigen(msg->pose.pose.position, data_.state.position);
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, data_.state.quaternion);
  tf::vectorMsgToEigen(msg->twist.twist.linear, data_.state.velocity);

  ROS_INFO("%f, %f, %f", data_.state.position.x(), data_.state.position.y(), data_.state.position.z());
}

void DroneCompanion::Run() {

  // Initialize the agent
  agent_.Initialize();

  ros::Rate run_ros_loop_at(loop_rate_);
  while (ros::ok()) {
    ros::spinOnce();
    Clock();
    ROS_INFO("loop freq: %f",1./data_.time_.dt_s);
    
    agent_.Run();

    PublishTopics();
    run_ros_loop_at.sleep();
  }

  ROS_WARN("Shutting down %s drone companion node!", ns_.c_str());
}


void DroneCompanion::PublishTopics() {
  // Yet to be implemented
}

} // namespace kite_ros
