#ifndef FPOWR_INITIAL_GUESS_EXTRACTOR_H_
#define FPOWR_INITIAL_GUESS_EXTRACTOR_H_ 

#include <Eigen/Dense>
#include <functional>
#include <towr/variables/spline_holder.h>
#include <fpowr/InitialGuessArray.h>

namespace fpowr {

struct InitialGuessStruct {
  double time;
  Eigen::VectorXd state;
  Eigen::VectorXd controls;
};

void ExtractInitialGuess(const towr::SplineHolder& spline_holder, double t, InitialGuessStruct& initial_guess) {
  initial_guess.time = t;

  // Assuming 6D base state (3D position + 3D orientation) and their velocities
  initial_guess.state = Eigen::VectorXd::Zero(12);
  initial_guess.state.segment<3>(0) = spline_holder.base_linear_->GetPoint(t).p(); // Base position
  initial_guess.state.segment<3>(3) = spline_holder.base_angular_->GetPoint(t).p(); // Base orientation
  initial_guess.state.segment<3>(6) = spline_holder.base_linear_->GetPoint(t).v(); // Base linear velocity
  initial_guess.state.segment<3>(9) = spline_holder.base_angular_->GetPoint(t).v(); // Base angular velocity

  // Assuming 3D accelerations, joint torques (set to zero), and forces for 4 end-effectors
  initial_guess.controls = Eigen::VectorXd::Zero(36);
  for (size_t i = 0; i < spline_holder.ee_force_.size(); ++i) {
    initial_guess.controls.segment<3>(i * 3) = spline_holder.ee_motion_.at(i)->GetPoint(t).a(); // End-effector accelerations
    initial_guess.controls.segment<3>(12 + i * 3) = Eigen::VectorXd::Zero(3); // Joint torques (set to zero)
    initial_guess.controls.segment<3>(24 + i * 3) = spline_holder.ee_force_.at(i)->GetPoint(t).p(); // Generalized forces
  }
}

void ExtractInitialGuesses(const fpowr::FootstepPlanGoalConstPtr &args, const towr::SplineHolder& spline_holder, fpowr::InitialGuessArray& initial_guess_array_msg) {
  for (auto time : args->state_sample_times) {
    InitialGuessStruct initial_guess;
    ExtractInitialGuess(spline_holder, time, initial_guess);

    fpowr::InitialGuess initial_guess_msg;
    initial_guess_msg.time = initial_guess.time;
    initial_guess_msg.state = std::vector<double>(initial_guess.state.data(), initial_guess.state.data() + initial_guess.state.size());
    initial_guess_msg.controls = std::vector<double>(initial_guess.controls.data(), initial_guess.controls.data() + initial_guess.controls.size());

    initial_guess_array_msg.initial_guesses.push_back(initial_guess_msg);
  }
}

} // namespace fpowr

#endif /* FPOWR_INITIAL_GUESS_EXTRACTOR_H_ */