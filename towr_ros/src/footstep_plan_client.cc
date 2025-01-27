#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <towr_ros/FootstepPlanAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_footstep_plan");

  // Create the action client
  // True causes the client to spin its own thread
  actionlib::SimpleActionClient<towr_ros::FootstepPlanAction> ac("footstep_plan", true);

  ROS_INFO("Waiting for action server to start.");
  // Wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // Send a goal to the action
  towr_ros::FootstepPlanGoal goal;
  // TODO
  ac.sendGoal(goal);

  // Wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
    ac.cancelGoal();
  }

  return 0;
}