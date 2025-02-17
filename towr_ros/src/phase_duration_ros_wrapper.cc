#include <towr_ros/PhaseDurationsRosWrapper.h>

namespace towr_ros {

towr_ros::PhaseDurations PhaseDurationsRosWrapper::ToRosMessage(const towr::PhaseDurations& phase_durations) {
  towr_ros::PhaseDurations msg;
  msg.durations = phase_durations.GetPhaseDurations();
  msg.initial_contact_state = phase_durations.initial_contact_state_;
  msg.min_phase_duration = phase_durations.phase_duration_bounds_.lower_;
  msg.max_phase_duration = phase_durations.phase_duration_bounds_.upper_;
  return msg;
}

towr::PhaseDurations PhaseDurationsRosWrapper::FromRosMessage(const towr_ros::PhaseDurations& msg) {
  towr::PhaseDurations phase_durations(0, msg.durations, msg.initial_contact_state, msg.min_phase_duration, msg.max_phase_duration);
  return phase_durations;
}

} // namespace towr_ros