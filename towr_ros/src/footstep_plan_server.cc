#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <towr_ros/FootstepPlanAction.h>
#include <towr/terrain/grid_height_map.h>
#include <towr/terrain/height_map_from_csv.h>
#include <towr/nlp_formulation.h>
#include <towr/initialization/gait_generator.h>
#include <towr/models/endeffector_mappings.h>
#include <xpp_states/convert.h>
#include <ifopt/ipopt_solver.h>
#include <geometry_msgs/PoseStamped.h>

#include <cpptrace/from_current.hpp>
#include <boost/stacktrace.hpp>

class FootstepPlanAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<towr_ros::FootstepPlanAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  towr_ros::FootstepPlanFeedback feedback_;
  towr_ros::FootstepPlanResult result_;

  // Publishers for start and goal poses
  ros::Publisher start_pose_pub_;
  ros::Publisher goal_pose_pub_;

public:

  FootstepPlanAction(std::string name) :
    as_(nh_, name, boost::bind(&FootstepPlanAction::executeCB, this, _1), false),
    action_name_(name)
  {
    // Initialize publishers
    start_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("start_pose", 1);
    goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);

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
    // - replace formulation.terrain_ = std::make_shared<HeightMapFromCSV>(grid_csv); with my implementation (grid_height_map.h)
    // - ensure the correct robot model is used
    // - figure out how get output in the correct format (towr_ros::FootstepPlanResult)
    //   - make another function to extract the footstep planes from the SRB trajectory and terrain

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
    bool optimize_gait = true;
    // const std::string grid_csv = "placeholder";
    float total_duration = 2; // seconds

    // Set up the NLP
    towr::NlpFormulation formulation;

    // terrain
    // TODO: Change to use grid_height_map.h
    // formulation.terrain_ = std::make_shared<HeightMapFromCSV>(grid_csv);
    // auto terrain_ptr = boost::make_shared<const convex_plane_decomposition_msgs::PlanarTerrain>(args->terrain);
    // formulation.terrain_ = std::make_shared<Grid>(*terrain_ptr);
    formulation.terrain_ = std::make_shared<Grid>(args->terrain);

    // Kinematic limits and dynamic parameters
    formulation.model_ = towr::RobotModel(towr::RobotModel::Go1);

    // initial position
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
    // TODO: are the XYZ or xyz euler angles?

    // TODO: can we set the goal ee positions? Do we want to?

    // desired goal state
    // formulation.final_base_.lin.at(towr::kPos) << x_final, y_final, -nominal_stance_B.front().z() + z_ground;
    formulation.final_base_.lin.at(towr::kPos) << args->goal_state.trunk_pose.position.x, args->goal_state.trunk_pose.position.y, args->goal_state.trunk_pose.position.z;
    Eigen::Quaterniond q_goal(
      args->goal_state.trunk_pose.orientation.w,
      args->goal_state.trunk_pose.orientation.x,
      args->goal_state.trunk_pose.orientation.y,
      args->goal_state.trunk_pose.orientation.z
    );
    formulation.final_base_.ang.at(towr::kPos) = q_goal.toRotationMatrix().eulerAngles(0, 1, 2);
    // TODO: are the XYZ or xyz euler angles?

    // Parameters defining contact sequence and default durations. We use
    // a GaitGenerator with some predifined gaits
    auto gait_gen_ = towr::GaitGenerator::MakeGaitGenerator(4);
    // TODO: this sould probably be passed in as a default argument or something.
    //       Maybe expose this option as a part of the action goal?
    auto id_gait = static_cast<towr::GaitGenerator::Combos>(0); // 0=walk, 1=flying trot, 2=pace, 3=bound, 4=gallop
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

    // IPOPT
    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
    solver->SetOption("hessian_approximation", "limited-memory");
    solver->SetOption("acceptable_iter", 15);
    solver->SetOption("acceptable_tol", 1e-3);
    solver->SetOption("max_iter", 3000);
    solver->SetOption("max_cpu_time", 10.0);
    solver->SetOption("tol", 1e-4);
    solver->SetOption("print_level", 5); // For debugging
    
    /* Which linear solver to use. Mumps is default because it comes with the
    * precompiled ubuntu binaries. However, the coin-hsl solvers can be
    * significantly faster and are free for academic purposes. They can be
    * downloaded here: http://www.hsl.rl.ac.uk/ipopt/ and must be compiled
    * into your IPOPT libraries. Then you can use the additional strings:
    * "ma27, ma57, ma77, ma86, ma97" here.
    */
    solver->SetOption("linear_solver", "mumps");

    // SNOPT
    // TODO: Currently doesn't work
    // auto solver = std::make_shared<ifopt::SnoptSolver>();
    // solver->SetOption("Print file", "snopt.out"); // Output file for SNOPT
    // solver->SetOption("Major iterations limit", 500);
    // solver->SetOption("Minor iterations limit", 1000);
    // solver->SetOption("Iterations limit", 1500);
    // solver->SetOption("Major optimality tolerance", 1e-4);
    // solver->SetOption("Major feasibility tolerance", 1e-6);
    // solver->SetOption("Minor feasibility tolerance", 1e-6);
    // solver->SetOption("Verify level", 0);

    // Solve!
    solver->Solve(nlp);

    return towr_ros::FootstepPlanResult();
  }

  void executeCB(const towr_ros::FootstepPlanGoalConstPtr &goal)
  {
    ROS_INFO("%s: Executing", action_name_.c_str());

    CPPTRACE_TRY {
      result_ = execute(goal);
    } CPPTRACE_CATCH(const std::exception& e) {
      // ROS_ERROR("%s: Exception caught trace: %s\n%s", action_name_.c_str(), e.what(), cpptrace::from_current_exception());
      ROS_ERROR("%s: Exception caught trace: %s", action_name_.c_str(), e.what());
      cpptrace::from_current_exception().print();
      as_.setAborted(result_, e.what());
      ROS_INFO("%s: Aborted", action_name_.c_str());
      return;
    }
    // try
    // {
    //   result_ = execute(goal);
    // }
    // catch(const std::exception& e)
    // {
    //   // ROS_ERROR("%s: Exception caught: %s", action_name_.c_str(), e.what());

    //   ROS_ERROR("%s: Exception caught trace: %s\n%s", action_name_.c_str(), e.what(), boost::stacktrace::to_string(boost::stacktrace::stacktrace()).c_str());
    //   as_.setAborted(result_, e.what());
    //   ROS_INFO("%s: Aborted", action_name_.c_str());
    //   return;
    // }

    // Set the action state to succeeded
    as_.setSucceeded(result_);
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