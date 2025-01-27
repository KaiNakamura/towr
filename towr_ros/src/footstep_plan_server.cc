#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <towr_ros/FootstepPlanAction.h>

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

  void executeCB(const towr_ros::FootstepPlanGoalConstPtr &goal)
  {
    ROS_INFO("%s: Executing footstep planner", action_name_.c_str());

    // TODO: Call towr
    // result_ = towr.doStuff();

    // Set the action state to succeeded
    as_.setSucceeded(result_);
    ROS_INFO("%s: Footstep planner succeeded", action_name_.c_str());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footstep_plan");

  FootstepPlanAction footstep_plan("footstep_plan");
  ros::spin();

  return 0;
}