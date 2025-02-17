#include <towr_ros/SplineHolderRosWrapper.h>

namespace towr_ros {

towr_ros::SplineHolder SplineHolderRosWrapper::ToRosMessage(const towr::SplineHolder& spline_holder) {
  towr_ros::SplineHolder msg;
  msg.base_linear = NodeSplineRosWrapper::ToRosMessage(*spline_holder.base_linear_);
  msg.base_angular = NodeSplineRosWrapper::ToRosMessage(*spline_holder.base_angular_);
  for (const auto& ee_motion : spline_holder.ee_motion_) {
    msg.ee_motion.push_back(NodeSplineRosWrapper::ToRosMessage(*ee_motion));
  }
  for (const auto& ee_force : spline_holder.ee_force_) {
    msg.ee_force.push_back(NodeSplineRosWrapper::ToRosMessage(*ee_force));
  }
  for (const auto& phase_duration : spline_holder.phase_durations_) {
    msg.phase_durations.push_back(PhaseDurationsRosWrapper::ToRosMessage(*phase_duration));
  }
  msg.ee_durations_change = spline_holder.ee_durations_change_;
  return msg;
}

towr::SplineHolder SplineHolderRosWrapper::FromRosMessage(const towr_ros::SplineHolder& msg) {
  towr::SplineHolder spline_holder;
  spline_holder.base_linear_ = std::make_shared<towr::NodeSpline>(NodeSplineRosWrapper::FromRosMessage(msg.base_linear));
  spline_holder.base_angular_ = std::make_shared<towr::NodeSpline>(NodeSplineRosWrapper::FromRosMessage(msg.base_angular));
  for (const auto& ee_motion_msg : msg.ee_motion) {
    spline_holder.ee_motion_.push_back(std::make_shared<towr::NodeSpline>(NodeSplineRosWrapper::FromRosMessage(ee_motion_msg)));
  }
  for (const auto& ee_force_msg : msg.ee_force) {
    spline_holder.ee_force_.push_back(std::make_shared<towr::NodeSpline>(NodeSplineRosWrapper::FromRosMessage(ee_force_msg)));
  }
  for (const auto& phase_duration_msg : msg.phase_durations) {
    spline_holder.phase_durations_.push_back(std::make_shared<towr::PhaseDurations>(PhaseDurationsRosWrapper::FromRosMessage(phase_duration_msg)));
  }
  spline_holder.ee_durations_change_ = msg.ee_durations_change;
  return spline_holder;
}

} // namespace towr_ros