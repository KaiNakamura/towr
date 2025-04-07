#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <towr/models/go1/go1_model.h>
#include <fpowr/FootstepPlanAction.h>
#include <fpowr/InitialGuessArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/foreach.hpp>

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

fpowr::SingleRigidBody createSingleRigidBodyState(double x, double y, double z, double w, double ox, double oy, double oz, const towr::KinematicModel& kinematic_model) {
  fpowr::SingleRigidBody state;
  state.trunk_pose = createTrunkPose(x, y, z, w, ox, oy, oz);
  state.LF_ee_point = createEndEffectorPoint(state.trunk_pose, kinematic_model.GetNominalStanceInBase().at(towr::LF));
  state.RF_ee_point = createEndEffectorPoint(state.trunk_pose, kinematic_model.GetNominalStanceInBase().at(towr::RF));
  state.LH_ee_point = createEndEffectorPoint(state.trunk_pose, kinematic_model.GetNominalStanceInBase().at(towr::LH));
  state.RH_ee_point = createEndEffectorPoint(state.trunk_pose, kinematic_model.GetNominalStanceInBase().at(towr::RH));
  return state;
}

void publishEndEffectorPositions(ros::Publisher& pub, const geometry_msgs::Point& point, const std::string& frame_id) {
  geometry_msgs::PointStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.point = point;
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "footstep_plan_client");

  ros::NodeHandle nh;

  // Parse command-line arguments for start and goal states
  if (argc < 13) {
    ROS_ERROR("Usage: rosrun towr_ros footstep_plan_client "
              "<start_x> <start_y> <start_z> <start_roll> <start_pitch> <start_yaw> "
              "<goal_x> <goal_y> <goal_z> <goal_roll> <goal_pitch> <goal_yaw>");
    return 1;
  }

  double start_x = std::stod(argv[1]);
  double start_y = std::stod(argv[2]);
  double start_z = std::stod(argv[3]);
  double start_roll = std::stod(argv[4]);
  double start_pitch = std::stod(argv[5]);
  double start_yaw = std::stod(argv[6]);

  double goal_x = std::stod(argv[7]);
  double goal_y = std::stod(argv[8]);
  double goal_z = std::stod(argv[9]);
  double goal_roll = std::stod(argv[10]);
  double goal_pitch = std::stod(argv[11]);
  double goal_yaw = std::stod(argv[12]);

  // Create publishers for start and goal end-effector positions
  ros::Publisher lf_start_pub = nh.advertise<geometry_msgs::PointStamped>("lf_start", 1);
  ros::Publisher rf_start_pub = nh.advertise<geometry_msgs::PointStamped>("rf_start", 1);
  ros::Publisher lh_start_pub = nh.advertise<geometry_msgs::PointStamped>("lh_start", 1);
  ros::Publisher rh_start_pub = nh.advertise<geometry_msgs::PointStamped>("rh_start", 1);

  ros::Publisher lf_goal_pub = nh.advertise<geometry_msgs::PointStamped>("lf_goal", 1);
  ros::Publisher rf_goal_pub = nh.advertise<geometry_msgs::PointStamped>("rf_goal", 1);
  ros::Publisher lh_goal_pub = nh.advertise<geometry_msgs::PointStamped>("lh_goal", 1);
  ros::Publisher rh_goal_pub = nh.advertise<geometry_msgs::PointStamped>("rh_goal", 1);

  // Create a publisher for the footstep plan
  ros::Publisher footstep_plan_pub = nh.advertise<fpowr::FootstepPlan>("footstep_plan", 1);

  // Create a publisher for the initial guesses
  ros::Publisher initial_guess_pub = nh.advertise<fpowr::InitialGuessArray>("initial_guesses", 1);

  // Create the action client
  // True causes the client to spin its own thread
  actionlib::SimpleActionClient<fpowr::FootstepPlanAction> ac("footstep_plan", true);

  ROS_INFO("Waiting for action server to start.");
  // Wait for the action server to start
  ac.waitForServer(); // Will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  // Open the bag file
  rosbag::Bag bag;
  bag.open("/home/catkin_ws/src/towr/fpowr/bag/perception_stairs.bag", rosbag::bagmode::Read);

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
  fpowr::FootstepPlanGoal args;

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
  Eigen::Quaterniond start_quat = Eigen::AngleAxisd(start_yaw, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(start_pitch, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(start_roll, Eigen::Vector3d::UnitX());
  args.start_state = createSingleRigidBodyState(start_x, start_y, start_z,
                                                start_quat.w(), start_quat.x(), start_quat.y(), start_quat.z(),
                                                kinematic_model);

  // Define the goal state
  Eigen::Quaterniond goal_quat = Eigen::AngleAxisd(goal_yaw, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(goal_pitch, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(goal_roll, Eigen::Vector3d::UnitX());
  args.goal_state = createSingleRigidBodyState(goal_x, goal_y, goal_z,
                                               goal_quat.w(), goal_quat.x(), goal_quat.y(), goal_quat.z(),
                                               kinematic_model);

  args.state_sample_times = {0.0, 0.25, 0.5, 0.75, 1.0};

  // Publish the start end-effector positions
  publishEndEffectorPositions(lf_start_pub, args.start_state.LF_ee_point, "odom");
  publishEndEffectorPositions(rf_start_pub, args.start_state.RF_ee_point, "odom");
  publishEndEffectorPositions(lh_start_pub, args.start_state.LH_ee_point, "odom");
  publishEndEffectorPositions(rh_start_pub, args.start_state.RH_ee_point, "odom");

  // Publish the goal end-effector positions
  publishEndEffectorPositions(lf_goal_pub, args.goal_state.LF_ee_point, "odom");
  publishEndEffectorPositions(rf_goal_pub, args.goal_state.RF_ee_point, "odom");
  publishEndEffectorPositions(lh_goal_pub, args.goal_state.LH_ee_point, "odom");
  publishEndEffectorPositions(rh_goal_pub, args.goal_state.RH_ee_point, "odom");

  // Send the goal
  ac.sendGoal(args);

  // Wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());

    // Get the result
    fpowr::FootstepPlanResultConstPtr result = ac.getResult();

    // Publish the footstep plan
    footstep_plan_pub.publish(result->footstep_plan);

    // Publish the initial guesses
    initial_guess_pub.publish(result->initial_guesses);
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}