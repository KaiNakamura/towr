#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <towr_ros/FootstepPlanAction.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

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

  // Open the bag file
  rosbag::Bag bag;
  bag.open("/home/catkin_ws/src/towr/towr_ros/bag/perception_stairs.bag", rosbag::bagmode::Read);

  // Create a view for the PlanarTerrain messages
  rosbag::View view(bag, rosbag::TopicQuery("/convex_plane_decomposition_ros/planar_terrain"));

  // Get the first PlanarTerrain message
  convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr terrain_msg;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr terrain_msg = m.instantiate<convex_plane_decomposition_msgs::PlanarTerrain>();
    if (terrain_msg != nullptr)
    {
      break;
    }
  }

  bag.close();

  // Note: The "FootstepPlanGoal" is just the ROS Action type for the inputs to
  // the footstep planner
  // NOT to be confused with the actual "goal" state of the robot
  towr_ros::FootstepPlanGoal args;

  // Attempt to set the terrain
  if (terrain_msg != nullptr)
  {
    args.terrain = *terrain_msg;
  }
  else
  {
    ROS_ERROR("No PlanarTerrain message found in the bag file.");
    return 1;
  }

  // TODO: Robot start and goal states

  // Send the goal
  ac.sendGoal(args);

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