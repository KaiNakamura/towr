#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <towr_ros/FootstepPlanAction.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <towr/models/go1/go1_model.h>

geometry_msgs::Pose getStartTrunkPose() {
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  return pose;
}

geometry_msgs::Point createEndEffectorPoint(const Eigen::Vector3d& vec) {
  geometry_msgs::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();
  return point;
}

geometry_msgs::Pose getGoalTrunkPose() {
  geometry_msgs::Pose pose;
  pose.position.x = 2.0;
  pose.position.y = 0.0;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  return pose;
}

geometry_msgs::Point getGoalEndEffectorPoint(const Eigen::Vector3d& vec) {
  return createEndEffectorPoint(vec + Eigen::Vector3d(2.0, 0.0, 0.0));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "footstep_plan_client");

  // Create the action client
  // True causes the client to spin its own thread
  actionlib::SimpleActionClient<towr_ros::FootstepPlanAction> ac("footstep_plan", true);

  ROS_INFO("Waiting for action server to start.");
  // Wait for the action server to start
  ac.waitForServer(); // will wait for infinite time

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
    terrain_msg = m.instantiate<convex_plane_decomposition_msgs::PlanarTerrain>();
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

  // Use Go1KinematicModel to get the start and goal states
  towr::Go1KinematicModel kinematic_model;

  // Define the start state
  towr_ros::SingleRigidBody start_state;
  start_state.trunk_pose = getStartTrunkPose();
  start_state.LF_ee_point = createEndEffectorPoint(kinematic_model.GetNominalStanceInBase().at(towr::LF));
  start_state.RF_ee_point = createEndEffectorPoint(kinematic_model.GetNominalStanceInBase().at(towr::RF));
  start_state.LH_ee_point = createEndEffectorPoint(kinematic_model.GetNominalStanceInBase().at(towr::LH));
  start_state.RH_ee_point = createEndEffectorPoint(kinematic_model.GetNominalStanceInBase().at(towr::RH));

  // Define the goal state
  towr_ros::SingleRigidBody goal_state;
  goal_state.trunk_pose = getGoalTrunkPose();
  goal_state.LF_ee_point = getGoalEndEffectorPoint(kinematic_model.GetNominalStanceInBase().at(towr::LF));
  goal_state.RF_ee_point = getGoalEndEffectorPoint(kinematic_model.GetNominalStanceInBase().at(towr::RF));
  goal_state.LH_ee_point = getGoalEndEffectorPoint(kinematic_model.GetNominalStanceInBase().at(towr::LH));
  goal_state.RH_ee_point = getGoalEndEffectorPoint(kinematic_model.GetNominalStanceInBase().at(towr::RH));

  // Set the start and goal states in the args
  args.start_state = start_state;
  args.goal_state = goal_state;

  // Send the goal
  ac.sendGoal(args);

  // Wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}