#include <ros/ros.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <grid_map_msgs/GridMap.h>

class GridMapRepublisher {
public:
    GridMapRepublisher() {
        // Initialize the node handle
        nh_ = ros::NodeHandle();

        // Create a publisher for the GridMap message
        gridmap_pub_ = nh_.advertise<grid_map_msgs::GridMap>("convex_plane_decomposition_ros/filtered_map", 10);

        // Create a subscriber for the PlanarTerrain message
        planar_terrain_sub_ = nh_.subscribe("convex_plane_decomposition_ros/planar_terrain", 10, &GridMapRepublisher::planarTerrainCallback, this);
    }

    void planarTerrainCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg) {
        // Extract the GridMap from the PlanarTerrain message
        grid_map_msgs::GridMap gridmap = msg->gridmap;

        // Republish the GridMap
        gridmap_pub_.publish(gridmap);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher gridmap_pub_;
    ros::Subscriber planar_terrain_sub_;
};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "gridmap_republisher");

    // Create an instance of the GridMapRepublisher class
    GridMapRepublisher republisher;

    // Spin to keep the node running
    ros::spin();

    return 0;
}