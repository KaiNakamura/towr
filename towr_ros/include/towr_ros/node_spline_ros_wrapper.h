#ifndef TOWR_ROS_NODE_SPLINE_ROS_WRAPPER_H_
#define TOWR_ROS_NODE_SPLINE_ROS_WRAPPER_H_

#include <towr/variables/node_spline.h>
#include <towr_ros/NodeSpline.h>

namespace towr_ros {

class NodeSplineRosWrapper {
public:
  static towr_ros::NodeSpline ToRosMessage(const towr::NodeSpline& node_spline);
  static towr::NodeSpline FromRosMessage(const towr_ros::NodeSpline& msg);
};

} // namespace towr_ros

#endif // TOWR_ROS_NODE_SPLINE_ROS_WRAPPER_H_