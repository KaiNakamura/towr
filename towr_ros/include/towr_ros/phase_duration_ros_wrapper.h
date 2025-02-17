#ifndef TOWR_ROS_PHASE_DURATIONS_ROS_WRAPPER_H_
#define TOWR_ROS_PHASE_DURATIONS_ROS_WRAPPER_H_

#include <towr/variables/phase_durations.h>
#include <towr_ros/PhaseDurations.h>

namespace towr_ros {

class PhaseDurationsRosWrapper {
public:
  static towr_ros::PhaseDurations ToRosMessage(const towr::PhaseDurations& phase_durations);
  static towr::PhaseDurations FromRosMessage(const towr_ros::PhaseDurations& msg);
};

} // namespace towr_ros

#endif // TOWR_ROS_PHASE_DURATIONS_ROS_WRAPPER_H_