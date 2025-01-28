#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <towr_ros/FootstepPlanAction.h>
#include <towr/terrain/grid_height_map.h>

#include <towr/terrain/grid_height_map.h>
#include <towr/terrain/Grid.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>

#include <towr/initialization/gait_generator.h>
#include <towr/models/endeffector_mappings.h>

class FootstepPlanAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<towr_ros::FootstepPlanAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  towr_ros::FootstepPlanFeedback feedback_;
  towr_ros::FootstepPlanResult result_;

public:

  FootstepPlanAction(std::string name) :
    as_(nh_, name, boost::bind(&FootstepPlanAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FootstepPlanAction(void)
  {
  }

  towr_ros::FootstepPlanResult execute(const towr_ros::FootstepPlanGoalConstPtr &args)
  {
    // implement code from:
    // https://github.com/opsullivan85/RBE550-Group-Project/blob/main/src/quadruped_drake/towr/trunk_mpc.cpp
    // - replace argv arguments with goal arguments
    // - replace std::cout with ROS_INFO, etc
    // - remove unnecessary arguments - gait_type, optimize_gait, etc
    // - replace formulation.terrain_ = std::make_shared<Grid>(grid_csv); with my implementation (grid_height_map.h)
    // - ensure the correct robot model is used
    // - figure out how get output in the correct format (towr_ros::FootstepPlanResult)
    //   - make another function to extract the footstep planes from the SRB trajectory and terrain

    // Set up the NLP
    NlpFormulation formulation;

    // terrain
    formulation.terrain_ = std::make_shared<Grid>(grid_csv);

    // Kinematic limits and dynamic parameters
    formulation.model_ = RobotModel(RobotModel::MiniCheetah);

    // initial position
    auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
    nominal_stance_B.at(LF) << args->start.LF_ee_point.x, args->start.LF_ee_point.y, args->start.LF_ee_point.z;
    nominal_stance_B.at(RF) << args->start.RF_ee_point.x, args->start.RF_ee_point.y, args->start.RF_ee_point.z;
    nominal_stance_B.at(LH) << args->start.LH_ee_point.x, args->start.LH_ee_point.y, args->start.LH_ee_point.z;
    nominal_stance_B.at(RH) << args->start.RH_ee_point.x, args->start.RH_ee_point.y, args->start.RH_ee_point.z;
    
    // TODO: Can we / do we need to set velocity values for the states?

    formulation.initial_base_.lin.at(towr::kPos) << args->start.trunk_pose.position.x, args->start.trunk_pose.position.y, args->start.trunk_pose.position.z;
    formulation.initial_base_.ang.at(towr::kPos) << 0, 0, 0; // TODO: how to convert from quaternion to xyz euler angles?
    // TODO: are the XYZ or xyz euler angles?

    // TODO: can we set the goal ee positions? Do we want to?

    // desired goal state
    // formulation.final_base_.lin.at(towr::kPos) << x_final, y_final, -nominal_stance_B.front().z() + z_ground;
    formulation.final_base_.lin.at(towr::kPos) << args->goal.trunk_pose.position.x, args->goal.trunk_pose.position.y, args->goal.trunk_pose.position.z;
    formulation.final_base_.ang.at(towr::kPos) << 0, 0, 0; // TODO: how to convert from quaternion to xyz euler angles?
    // TODO: are the XYZ or xyz euler angles?

    // Parameters defining contact sequence and default durations. We use
    // a GaitGenerator with some predifined gaits
    auto gait_gen_ = GaitGenerator::MakeGaitGenerator(4);
    // TODO: this sould probably be passed in as a default argument or something.
    //       Maybe expose this option as a part of the action goal?
    auto id_gait = static_cast<GaitGenerator::Combos>(0); // 0=walk, 1=flying trot, 2=pace, 3=bound, 4=gallop
    gait_gen_->SetCombo(id_gait);
    for (int ee = 0; ee < 4; ++ee)
    {
        formulation.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(total_duration, ee));
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
    SplineHolder solution;
    for (auto c : formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto c : formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto c : formulation.GetCosts())
        nlp.AddCostSet(c);

    // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
    // solver->SetOption("derivative_test", "first-order");
    // TODO: Fine tune these parameters to suit this use case
    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
    solver->SetOption("hessian_approximation", "limited-memory");
    solver->SetOption("acceptable_iter", 15);
    solver->SetOption("acceptable_tol", 1e-3);
    solver->SetOption("max_iter", 500);
    solver->SetOption("max_cpu_time", 2.0);
    solver->SetOption("tol", 1e-4);
    solver->Solve(nlp);

    return towr_ros::FootstepPlanResult();
  }

  void executeCB(const towr_ros::FootstepPlanGoalConstPtr &goal)
  {
    ROS_INFO("%s: Executing footstep planner", action_name_.c_str());

    try
    {
      result_ = execute(goal);
    }
    catch(const std::exception& e)
    {
      as_.setAborted(result_, e.what());
      ROS_INFO("%s: Footstep planner aborted", action_name_.c_str());
      return;
    }

    // Set the action state to succeeded
    as_.setSucceeded(result_);
    ROS_INFO("%s: Footstep planner succeeded", action_name_.c_str());
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