#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <towr_ros/FootstepPlanAction.h>
#include <towr/terrain/grid_height_map.h>

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

  towr_ros::FootstepPlanResult execute(const towr_ros::FootstepPlanGoalConstPtr &goal)
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