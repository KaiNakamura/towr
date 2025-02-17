#include <towr_ros/NodeSplineRosWrapper.h>

namespace towr_ros {

towr_ros::NodeSpline NodeSplineRosWrapper::ToRosMessage(const towr::NodeSpline& node_spline) {
  towr_ros::NodeSpline msg;
  msg.polynomial_durations = node_spline.GetPolyDurations();
  for (const auto& poly : node_spline.cubic_polys_) {
    for (const auto& coeff : poly.coefficients) {
      msg.coefficients.push_back(coeff);
    }
  }
  return msg;
}

towr::NodeSpline NodeSplineRosWrapper::FromRosMessage(const towr_ros::NodeSpline& msg) {
  towr::NodeSpline node_spline(nullptr, msg.polynomial_durations);
  int coeff_index = 0;
  for (auto& poly : node_spline.cubic_polys_) {
    for (auto& coeff : poly.coefficients) {
      coeff = msg.coefficients[coeff_index++];
    }
  }
  return node_spline;
}

} // namespace towr_ros