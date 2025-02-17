#ifndef TOWR_ROS_SPLINE_HOLDER_ROS_WRAPPER_H_
#define TOWR_ROS_SPLINE_HOLDER_ROS_WRAPPER_H_

#include <towr/variables/spline_holder.h>
#include <towr_ros/SplineHolder.h>
#include <towr_ros/NodeSplineRosWrapper.h>
#include <towr_ros/PhaseDurationsRosWrapper.h>

namespace towr_ros {

class SplineHolderRosWrapper {
public:
  static towr_ros::SplineHolder ToRosMessage(const towr::SplineHolder& spline_holder);
  static towr::SplineHolder FromRosMessage(const towr_ros::SplineHolder& msg);
};

} // namespace towr_ros

#endif // TOWR_ROS_SPLINE_HOLDER_ROS_WRAPPER_H_