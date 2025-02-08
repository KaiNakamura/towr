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
#include <towr/models/examples/hyq_model.h>

geometry_msgs::Pose createTrunkPose(double x, double y, double z, double w, double ox, double oy, double oz) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = w;
  pose.orientation.x = ox;
  pose.orientation.y = oy;
  pose.orientation.z = oz;
  return pose;
}

geometry_msgs::Point createEndEffectorPoint(const geometry_msgs::Pose& trunk_pose, const Eigen::Vector3d& nominal_stance) {
  geometry_msgs::Point point;
  point.x = trunk_pose.position.x + nominal_stance.x();
  point.y = trunk_pose.position.y + nominal_stance.y();
  point.z = trunk_pose.position.z + nominal_stance.z();
  return point;
}

towr_ros::SingleRigidBody createSingleRigidBodyState(double x, double y, double z, double w, double ox, double oy, double oz, const towr::KinematicModel& kinematic_model) {
  towr_ros::SingleRigidBody state;
  state.trunk_pose = createTrunkPose(x, y, z, w, ox, oy, oz);
  state.LF_ee_point = createEndEffectorPoint(state.trunk_pose, kinematic_model.GetNominalStanceInBase().at(towr::LF));
  state.RF_ee_point = createEndEffectorPoint(state.trunk_pose, kinematic_model.GetNominalStanceInBase().at(towr::RF));
  state.LH_ee_point = createEndEffectorPoint(state.trunk_pose, kinematic_model.GetNominalStanceInBase().at(towr::LH));
  state.RH_ee_point = createEndEffectorPoint(state.trunk_pose, kinematic_model.GetNominalStanceInBase().at(towr::RH));
  return state;
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
  // towr::HyqKinematicModel kinematic_model;

  // Define the start state
  args.start_state = createSingleRigidBodyState(-1.0, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0, kinematic_model);

  // Define the goal state
  args.goal_state = createSingleRigidBodyState(0.0, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0, kinematic_model);

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