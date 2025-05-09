#ifndef FPOWR_FOOTSTEP_PLAN_EXTRACTOR_H_
#define FPOWR_FOOTSTEP_PLAN_EXTRACTOR_H_

#include <ros/ros.h>
#include <towr/variables/spline_holder.h>
#include <fpowr/FootstepPlanGoal.h>
#include <fpowr/FootstepPlan.h>
#include <fpowr/fpowr_xpp_ee_map.h>
#include <fpowr/nearest_plane_lookup.h>
#include <xpp_states/robot_state_cartesian.h>
#include <xpp_states/endeffectors.h>
#include <xpp_states/state.h>
#include <xpp_states/convert.h>

namespace fpowr {

using XppVec = std::vector<xpp::RobotStateCartesian>;

XppVec GetTrajectory(const towr::SplineHolder& solution, double dt)
{
  XppVec trajectory;
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();

  towr::EulerConverter base_angular(solution.base_angular_);

  while (t <= T + 1e-5)
  {
    int n_ee = solution.ee_motion_.size();
    xpp::RobotStateCartesian state(n_ee);

    state.base_.lin = fpowr::ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (int ee_fpowr = 0; ee_fpowr < n_ee; ++ee_fpowr)
    {
      int ee_xpp = fpowr::ToXppEndeffector(n_ee, ee_fpowr).first;

      state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_fpowr)->IsContactPhase(t);
      state.ee_motion_.at(ee_xpp) = fpowr::ToXpp(solution.ee_motion_.at(ee_fpowr)->GetPoint(t));
      state.ee_forces_.at(ee_xpp) = solution.ee_force_.at(ee_fpowr)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += dt;
  }

  return trajectory;
}

bool HasEndEffectorContactChanged(const xpp::EndeffectorsContact& a, const xpp::EndeffectorsContact& b) {
  if (a.GetEECount() != b.GetEECount()) {
    ROS_DEBUG("Endeffector contact sizes do not match: %d vs %d", a.GetEECount(), b.GetEECount());
    return true;
  }

  for (auto ee : a.GetEEsOrdered()) {
    if (a.at(ee) != b.at(ee))
      return true;
  }

  return false;
}

void ExtractFootstepPlan(const fpowr::FootstepPlanGoalConstPtr &args, const towr::SplineHolder& solution, double time_horizon, fpowr::FootstepPlan& footstep_plan_msg)
{
  // Create a nearest_plane_lookup object
  // TODO: Are we freeing this anywhere?
  NearestPlaneLookup nearest_plane_lookup = NearestPlaneLookup(args->terrain);

  auto LF_start_plane = nearest_plane_lookup.GetNearestPlaneIndex((Eigen::VectorXd(2) << args->start_state.LF_ee_point.x, args->start_state.LF_ee_point.y).finished());
  auto RF_start_plane = nearest_plane_lookup.GetNearestPlaneIndex((Eigen::VectorXd(2) << args->start_state.RF_ee_point.x, args->start_state.RF_ee_point.y).finished());
  auto LH_start_plane = nearest_plane_lookup.GetNearestPlaneIndex((Eigen::VectorXd(2) << args->start_state.LH_ee_point.x, args->start_state.LH_ee_point.y).finished());
  auto RH_start_plane = nearest_plane_lookup.GetNearestPlaneIndex((Eigen::VectorXd(2) << args->start_state.RH_ee_point.x, args->start_state.RH_ee_point.y).finished());

  // Publish the start planes with ros info
  ROS_INFO("Start plane for LF: %d", LF_start_plane);
  ROS_INFO("Start plane for RF: %d", RF_start_plane);
  ROS_INFO("Start plane for LH: %d", LH_start_plane);
  ROS_INFO("Start plane for RH: %d", RH_start_plane);

  // Convert solution to discretized trajectory
  auto trajectory = GetTrajectory(solution, 0.01);

  // Iterate over trajectory states to find all footsteps
  XppVec footstepStates;
  xpp::RobotStateCartesian lastState(solution.ee_motion_.size());
  bool isFirstState = true;
  for (const auto& state : trajectory)
  {
    // If first state or ee contact has changed, add footstep state
    if (
      isFirstState ||
      HasEndEffectorContactChanged(state.ee_contact_, lastState.ee_contact_)
    ) {
      footstepStates.push_back(state);
      isFirstState = false;
    }

    // Update last state
    lastState = state;
  }

  // Populate footstep plan msg
  for (size_t i = 0; i < footstepStates.size(); ++i) {
    const auto& state = footstepStates[i];
    fpowr::ContactDatum contact_datum;

    for (const auto& ee_contact : state.ee_contact_.GetEEsOrdered()) {
      int nearest_plane_index = -1; // Default to -1 if the end effector is in the air
      if (state.ee_contact_.at(ee_contact)) {
        // Get the nearest plane index for the current end effector
        nearest_plane_index = nearest_plane_lookup.GetNearestPlaneIndex(state.ee_motion_.at(ee_contact).p_);
      }

      contact_datum.contact_set.push_back(nearest_plane_index);
    }

    if (i < footstepStates.size() - 1) {
      // For each step except the last one, set the duration as the time until the next step
      contact_datum.duration = (footstepStates[i + 1].t_global_ - state.t_global_);
    } else {
      // For the last step, set the duration until the end of the time horizon
      contact_datum.duration = (time_horizon - state.t_global_);
    }

    footstep_plan_msg.contact_data.push_back(contact_datum);
  }
}

} // namespace fpowr 

#endif /* FPOWR_FOOTSTEP_PLAN_EXTRACTOR_H_ */