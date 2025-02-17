#ifndef TOWR_ROS_SPLINE_HOLDER_ROS_CONVERSION_H_
#define TOWR_ROS_SPLINE_HOLDER_ROS_CONVERSION_H_ 

#include <towr/variables/spline_holder.h>
#include <towr_ros/SplineHolder.h>
#include <towr_ros/NodeSpline.h>
#include <towr_ros/PhaseDurations.h>

namespace towr {

towr_ros::NodeSpline ToRosMessage(const NodeSpline::Ptr& node_spline) {
  towr_ros::NodeSpline msg;
  // Assuming NodeSpline has methods to get positions and velocities
  msg.positions = std::vector<double>(node_spline->GetPositions().data(), node_spline->GetPositions().data() + node_spline->GetPositions().size());
  msg.velocities = std::vector<double>(node_spline->GetVelocities().data(), node_spline->GetVelocities().data() + node_spline->GetVelocities().size());
  return msg;
}

towr_ros::PhaseDurations ToRosMessage(const PhaseDurations::Ptr& phase_durations) {
  towr_ros::PhaseDurations msg;
  msg.durations = phase_durations->GetDurations();
  return msg;
}

towr_ros::SplineHolder ToRosMessage(const SplineHolder& spline_holder) {
  towr_ros::SplineHolder msg;
  msg.base_linear = ToRosMessage(spline_holder.base_linear_);
  msg.base_angular = ToRosMessage(spline_holder.base_angular_);
  for (const auto& ee_motion : spline_holder.ee_motion_) {
    msg.ee_motion.push_back(ToRosMessage(ee_motion));
  }
  for (const auto& ee_force : spline_holder.ee_force_) {
    msg.ee_force.push_back(ToRosMessage(ee_force));
  }
  for (const auto& phase_duration : spline_holder.phase_durations_) {
    msg.phase_durations.push_back(ToRosMessage(phase_duration));
  }
  return msg;
}

NodeSpline::Ptr FromRosMessage(const towr_ros::NodeSpline& msg) {
  // Assuming NodeSpline has a constructor that takes positions and velocities
  return std::make_shared<NodeSpline>(msg.positions, msg.velocities);
}

PhaseDurations::Ptr FromRosMessage(const towr_ros::PhaseDurations& msg) {
  return std::make_shared<PhaseDurations>(msg.durations);
}

SplineHolder FromRosMessage(const towr_ros::SplineHolder& msg) {
  SplineHolder spline_holder;
  spline_holder.base_linear_ = FromRosMessage(msg.base_linear);
  spline_holder.base_angular_ = FromRosMessage(msg.base_angular);
  for (const auto& ee_motion_msg : msg.ee_motion) {
    spline_holder.ee_motion_.push_back(FromRosMessage(ee_motion_msg));
  }
  for (const auto& ee_force_msg : msg.ee_force) {
    spline_holder.ee_force_.push_back(FromRosMessage(ee_force_msg));
  }
  for (const auto& phase_duration_msg : msg.phase_durations) {
    spline_holder.phase_durations_.push_back(FromRosMessage(phase_duration_msg));
  }
  return spline_holder;
}

} // namespace towr

#endif /* TOWR_ROS_SPLINE_HOLDER_ROS_CONVERSION_H_ */