#ifndef TOWR_ROS_SPLINE_HOLDER_ROS_CONVERSION_H_
#define TOWR_ROS_SPLINE_HOLDER_ROS_CONVERSION_H_ 

#include <towr/variables/spline_holder.h>
#include <towr_ros/SplineHolder.h>
#include <towr_ros/NodeSpline.h>
#include <towr_ros/PhaseDurations.h>

namespace towr {

towr_ros::NodeSpline ToRosMessage(const NodeSpline::Ptr& node_spline) {
}

towr_ros::PhaseDurations ToRosMessage(const PhaseDurations::Ptr& phase_durations) {
}

towr_ros::SplineHolder ToRosMessage(const SplineHolder& spline_holder) {
}

NodeSpline::Ptr FromRosMessage(const towr_ros::NodeSpline& msg) {
}

PhaseDurations::Ptr FromRosMessage(const towr_ros::PhaseDurations& msg) {
}

SplineHolder FromRosMessage(const towr_ros::SplineHolder& msg) {
}

} // namespace towr

#endif /* TOWR_ROS_SPLINE_HOLDER_ROS_CONVERSION_H_ */