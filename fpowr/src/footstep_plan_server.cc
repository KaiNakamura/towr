#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <towr/nlp_formulation.h>
#include <towr/initialization/gait_generator.h>
#include <towr/models/endeffector_mappings.h>
#include <towr/terrain/grid_height_map.h>
#include <towr/terrain/height_map_from_csv.h>
#include <towr/variables/spline_holder.h>
#include <fpowr/FootstepPlanAction.h>
#include <fpowr/FootstepPlan.h>
#include <fpowr/footstep_plan_extractor.h>
#include <fpowr/fpowr_xpp_ee_map.h>
#include <fpowr/initial_guess_extractor.h>
#include <fpowr/nearest_plane_lookup.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/snopt_solver.h>
#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <tf/transform_datatypes.h>
#include <boost/stacktrace.hpp>
#include <cpptrace/from_current.hpp>

class FootstepPlanAction
{
protected:
  static constexpr double TIME_HORIZON = 2; // Seconds
  static constexpr double PLAYBACK_SPEED = 0.5; // Playback speed for urdf visualization

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<fpowr::FootstepPlanAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;

  // Create messages that are used to published feedback/result
  fpowr::FootstepPlanFeedback feedback_;
  fpowr::FootstepPlanResult result_;

  // Publishers
  ros::Publisher base_path_pub_;
  ros::Publisher lf_path_pub_;
  ros::Publisher rf_path_pub_;
  ros::Publisher lh_path_pub_;
  ros::Publisher rh_path_pub_;
  ros::Publisher start_pose_pub_;
  ros::Publisher goal_pose_pub_;
  ros::Publisher robot_state_pub_;

public:

  FootstepPlanAction(std::string name) :
    as_(nh_, name, boost::bind(&FootstepPlanAction::executeCB, this, _1), false),
    action_name_(name)
  {
    // Initialize publishers
    base_path_pub_ = nh_.advertise<nav_msgs::Path>("/base_path", 1);
    lf_path_pub_ = nh_.advertise<nav_msgs::Path>("/lf_path", 1);
    rf_path_pub_ = nh_.advertise<nav_msgs::Path>("/rf_path", 1);
    lh_path_pub_ = nh_.advertise<nav_msgs::Path>("/lh_path", 1);
    rh_path_pub_ = nh_.advertise<nav_msgs::Path>("/rh_path", 1);
    start_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("start_pose", 1);
    goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);
    robot_state_pub_ = nh_.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_desired, 1); // Initialize the publisher

    as_.start();
  }

  ~FootstepPlanAction(void)
  {
  }

  void publishPaths(const towr::SplineHolder& solution)
  {
    nav_msgs::Path base_path, lf_path, rf_path, lh_path, rh_path;
    base_path.header.frame_id = lf_path.header.frame_id = rf_path.header.frame_id = lh_path.header.frame_id = rh_path.header.frame_id = "base_link";
    base_path.header.stamp = lf_path.header.stamp = rf_path.header.stamp = lh_path.header.stamp = rh_path.header.stamp = ros::Time::now();

    double dt = 0.01;
    for (double t = 0.0; t <= solution.base_linear_->GetTotalTime(); t += dt)
    {
      geometry_msgs::PoseStamped base_pose, lf_pose, rf_pose, lh_pose, rh_pose;
      base_pose.pose.position.x = solution.base_linear_->GetPoint(t).p().x();
      base_pose.pose.position.y = solution.base_linear_->GetPoint(t).p().y();
      base_pose.pose.position.z = solution.base_linear_->GetPoint(t).p().z();
      base_path.poses.push_back(base_pose);

      lf_pose.pose.position.x = solution.ee_motion_.at(towr::LF)->GetPoint(t).p().x();
      lf_pose.pose.position.y = solution.ee_motion_.at(towr::LF)->GetPoint(t).p().y();
      lf_pose.pose.position.z = solution.ee_motion_.at(towr::LF)->GetPoint(t).p().z();
      lf_path.poses.push_back(lf_pose);

      rf_pose.pose.position.x = solution.ee_motion_.at(towr::RF)->GetPoint(t).p().x();
      rf_pose.pose.position.y = solution.ee_motion_.at(towr::RF)->GetPoint(t).p().y();
      rf_pose.pose.position.z = solution.ee_motion_.at(towr::RF)->GetPoint(t).p().z();
      rf_path.poses.push_back(rf_pose);

      lh_pose.pose.position.x = solution.ee_motion_.at(towr::LH)->GetPoint(t).p().x();
      lh_pose.pose.position.y = solution.ee_motion_.at(towr::LH)->GetPoint(t).p().y();
      lh_pose.pose.position.z = solution.ee_motion_.at(towr::LH)->GetPoint(t).p().z();
      lh_path.poses.push_back(lh_pose);

      rh_pose.pose.position.x = solution.ee_motion_.at(towr::RH)->GetPoint(t).p().x();
      rh_pose.pose.position.y = solution.ee_motion_.at(towr::RH)->GetPoint(t).p().y();
      rh_pose.pose.position.z = solution.ee_motion_.at(towr::RH)->GetPoint(t).p().z();
      rh_path.poses.push_back(rh_pose);
    }

    base_path_pub_.publish(base_path);
    lf_path_pub_.publish(lf_path);
    rf_path_pub_.publish(rf_path);
    lh_path_pub_.publish(lh_path);
    rh_path_pub_.publish(rh_path);
  }

  void publishRobotState(const towr::SplineHolder& solution)
  {
    // Match dt of trajectory and playback rate
    ros::Rate rate(100 * PLAYBACK_SPEED);
    auto trajectory = fpowr::GetTrajectory(solution, 0.01 * PLAYBACK_SPEED);

    for (const auto& state : trajectory)
    {
      xpp_msgs::RobotStateCartesian msg = xpp::Convert::ToRos(state);
      robot_state_pub_.publish(msg);
      rate.sleep();
    }
  }

  towr::SplineHolder execute(const fpowr::FootstepPlanGoalConstPtr &args)
  {
    // Publish start and goal poses for visualization
    geometry_msgs::PoseStamped start_pose_msg;
    start_pose_msg.header.stamp = ros::Time::now();
    start_pose_msg.header.frame_id = "odom";
    start_pose_msg.pose = args->start_state.trunk_pose;
    start_pose_pub_.publish(start_pose_msg);

    geometry_msgs::PoseStamped goal_pose_msg;
    goal_pose_msg.header.stamp = ros::Time::now();
    goal_pose_msg.header.frame_id = "odom";
    goal_pose_msg.pose = args->goal_state.trunk_pose;
    goal_pose_pub_.publish(goal_pose_msg);

    // Some placeholders to get things building
    bool optimize_gait = false;
    // const std::string grid_csv = "placeholder";

    // Set up the NLP
    towr::NlpFormulation formulation;

    // Create terrain
    formulation.terrain_ = std::make_shared<Grid>(args->terrain);

    // Kinematic limits and dynamic parameters
    formulation.model_ = towr::RobotModel(towr::RobotModel::Go1);

    // Initial state
    auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
    nominal_stance_B.at(towr::LF) << args->start_state.LF_ee_point.x, args->start_state.LF_ee_point.y, args->start_state.LF_ee_point.z;
    nominal_stance_B.at(towr::RF) << args->start_state.RF_ee_point.x, args->start_state.RF_ee_point.y, args->start_state.RF_ee_point.z;
    nominal_stance_B.at(towr::LH) << args->start_state.LH_ee_point.x, args->start_state.LH_ee_point.y, args->start_state.LH_ee_point.z;
    nominal_stance_B.at(towr::RH) << args->start_state.RH_ee_point.x, args->start_state.RH_ee_point.y, args->start_state.RH_ee_point.z;
    formulation.initial_ee_W_ = nominal_stance_B;
    
    // TODO: Can we / do we need to set velocity values for the states?

    formulation.initial_base_.lin.at(towr::kPos) << args->start_state.trunk_pose.position.x, args->start_state.trunk_pose.position.y, args->start_state.trunk_pose.position.z;
    Eigen::Quaterniond q_start(
      args->start_state.trunk_pose.orientation.w,
      args->start_state.trunk_pose.orientation.x,
      args->start_state.trunk_pose.orientation.y,
      args->start_state.trunk_pose.orientation.z
    );
    formulation.initial_base_.ang.at(towr::kPos) = q_start.toRotationMatrix().eulerAngles(0, 1, 2);

    // Desired goal state
    formulation.final_base_.lin.at(towr::kPos) << args->goal_state.trunk_pose.position.x, args->goal_state.trunk_pose.position.y, args->goal_state.trunk_pose.position.z;
    Eigen::Quaterniond q_goal(
      args->goal_state.trunk_pose.orientation.w,
      args->goal_state.trunk_pose.orientation.x,
      args->goal_state.trunk_pose.orientation.y,
      args->goal_state.trunk_pose.orientation.z
    );
    formulation.final_base_.ang.at(towr::kPos) = q_goal.toRotationMatrix().eulerAngles(0, 1, 2);

    // Parameters defining contact sequence and default durations. We use
    // a GaitGenerator with some predifined gaits
    auto gait_gen_ = towr::GaitGenerator::MakeGaitGenerator(4);
    // TODO: this sould probably be passed in as a default argument or something.
    //       Maybe expose this option as a part of the action goal?
    auto id_gait = static_cast<towr::GaitGenerator::Combos>(1); // 0=walk, 1=flying trot, 2=pace, 3=bound, 4=gallop
    gait_gen_->SetCombo(id_gait);
    for (int ee = 0; ee < 4; ++ee)
    {
        formulation.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(TIME_HORIZON, ee));
        formulation.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
    }

    // Indicate whether to optimize over gaits as well
    if (optimize_gait)
    {
        formulation.params_.OptimizePhaseDurations();
    }

    // Add weighted cost on rotational velocity of base
    // formulation.params_.costs_.push_back({Parameters::CostName(1),1.0});

    // Initialize the nonlinear-programming problem with the variables,
    // constraints and costs.
    ifopt::Problem nlp;
    towr::SplineHolder solution;
    for (auto c : formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto c : formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto c : formulation.GetCosts())
        nlp.AddCostSet(c);

    // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
    // solver->SetOption("derivative_test", "first-order");
    // TODO: Fine tune these parameters to suit this use case

    // Ipopt
    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
    solver->SetOption("hessian_approximation", "limited-memory");
    solver->SetOption("acceptable_iter", 15);
    solver->SetOption("acceptable_tol", 1e-3);
    solver->SetOption("max_iter", 500);
    solver->SetOption("max_cpu_time", 5);
    solver->SetOption("tol", 1e-4);
    solver->SetOption("print_level", 5); // For debugging
    solver->SetOption("linear_solver", "ma57"); // Note: Mumps is default but is slow
    // solver->SetOption("linear_solver", "ma97"); // Note: Mumps is default but is slow

    // Snopt
    // auto solver = std::make_shared<ifopt::SnoptSolver>();
    // Default values (for reference)
    // solver->SetIntParameter("Major Print level", 1);
    // solver->SetIntParameter("Minor Print level", 1);
    // solver->SetIntParameter("Derivative option", 1);  // 1 = solver->will not calculate missing derivatives
    // solver->SetIntParameter("Verify level ", 3);
    // solver->SetIntParameter("Iterations limit", 200000);
    // solver->SetRealParameter("Major feasibility tolerance", 1.0e-4);  // target nonlinear constraint violation
    // solver->SetRealParameter("Minor feasibility tolerance", 1.0e-4);  // for satisfying the QP bounds
    // solver->SetRealParameter("Major optimality tolerance", 1.0e-2);  // target complementarity gap

    // solver->SetIntParameter("Major Print level", 1);
    // solver->SetIntParameter("Minor Print level", 1);
    // solver->SetIntParameter("Derivative option", 1);  // 1 = solver->will not calculate missing derivatives
    // solver->SetIntParameter("Verify level ", 3);
    // solver->SetIntParameter("Iterations limit", 200000);
    // solver->SetRealParameter("Major feasibility tolerance", 1.0e-4);  // target nonlinear constraint violation
    // solver->SetRealParameter("Minor feasibility tolerance", 1.0e-4);  // for satisfying the QP bounds
    // solver->SetRealParameter("Major optimality tolerance", 1.0e-2);  // target complementarity gap

    // Solve!
    try {
      solver->Solve(nlp);
    } catch (const std::exception& e) {
      ROS_ERROR("Exception caught during solver execution: %s", e.what());
    }

    return solution;
  }

  void executeCB(const fpowr::FootstepPlanGoalConstPtr &args)
  {
    ROS_INFO("%s: Executing", action_name_.c_str());
    result_ = fpowr::FootstepPlanResult(); // Initialize the result
    towr::SplineHolder solution;

    CPPTRACE_TRY {
      solution = execute(args);
    } CPPTRACE_CATCH(const std::exception& e) {
      ROS_ERROR("%s: Exception caught trace: %s", action_name_.c_str(), e.what());
      cpptrace::from_current_exception().print();
      as_.setAborted(result_, e.what());
      ROS_INFO("%s: Aborted", action_name_.c_str());
      return;
    }

    // With the solution, extract initial guesses and footstep plan
    fpowr::ExtractInitialGuesses(args, solution, result_.initial_guesses);
    fpowr::ExtractFootstepPlan(args, solution, TIME_HORIZON, result_.footstep_plan);

    // Set the action state to succeeded
    as_.setSucceeded(result_);

    publishPaths(solution);
    publishRobotState(solution);

    ROS_INFO("%s: Succeeded", action_name_.c_str());
    return;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footstep_plan");

  FootstepPlanAction footstep_plan("footstep_plan");
  ros::spin();

  return 0;
}