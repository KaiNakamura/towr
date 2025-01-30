#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <boost/foreach.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planar_terrain_from_bag_publisher");

  ros::NodeHandle n;

  ros::Publisher terrain_pub = n.advertise<convex_plane_decomposition_msgs::PlanarTerrain>("terrain", 100);

  // Open the bag file
  rosbag::Bag bag;
  bag.open("/home/catkin_ws/src/towr/towr_ros/bag/perception_stairs.bag", rosbag::bagmode::Read);

  // Create a view for the PlanarTerrain messages
  rosbag::View view(bag, rosbag::TopicQuery("/convex_plane_decomposition_ros/planar_terrain"));

  ros::Rate loop_rate(10);

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr terrain_msg = m.instantiate<convex_plane_decomposition_msgs::PlanarTerrain>();
    if (terrain_msg != nullptr)
    {
      terrain_pub.publish(terrain_msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  bag.close();

  return 0;
}