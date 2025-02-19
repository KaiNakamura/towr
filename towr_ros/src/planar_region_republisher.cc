#include <ros/ros.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

class PlanarRegionRepublisher {
public:
    PlanarRegionRepublisher() {
        // Initialize the node handle
        nh_ = ros::NodeHandle();

        // Create a publisher for the MarkerArray message
        planar_regions_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("planar_regions", 10);

        // Create a subscriber for the PlanarTerrain message
        planar_terrain_sub_ = nh_.subscribe("convex_plane_decomposition_ros/planar_terrain", 10, &PlanarRegionRepublisher::planarTerrainCallback, this);
    }

    void planarTerrainCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        // Iterate over each planar region
        for (const auto& region : msg->planarRegions) {
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "odom";
            marker.ns = "planar_regions";
            marker.id = id;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.01;

            // Set color based on fixed coloring scheme
            std::vector<float> rgb = getColorByIndex(id);
            marker.color.r = rgb[0];
            marker.color.g = rgb[1];
            marker.color.b = rgb[2];
            marker.color.a = 1.0;

            // Set the orientation to the identity quaternion
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Extract plane parameters
            tf::Quaternion q(region.plane_parameters.orientation.x,
                             region.plane_parameters.orientation.y,
                             region.plane_parameters.orientation.z,
                             region.plane_parameters.orientation.w);
            tf::Matrix3x3 R(q);

            // Process the points and close the loop
            for (size_t i = 0; i <= region.boundary.outer_boundary.points.size(); ++i) {
                const auto& point = region.boundary.outer_boundary.points[i % region.boundary.outer_boundary.points.size()];
                tf::Vector3 p_2d(point.x, point.y, 0.0);
                tf::Vector3 p_3d = R * p_2d + tf::Vector3(region.plane_parameters.position.x,
                                                          region.plane_parameters.position.y,
                                                          region.plane_parameters.position.z);

                geometry_msgs::Point p;
                p.x = p_3d.x();
                p.y = p_3d.y();
                // Adjust the z-coordinate slightly to align with the terrain
                // Not sure why this needs to be done, but otherwise it hovers over the terrain too much
                // Might have to do with the line thickness
                p.z = p_3d.z() - 0.02;
                marker.points.push_back(p);
            }

            // Add the marker to the marker array
            marker_array.markers.push_back(marker);

            // Increment id for the next region
            id++;
        }

        // Publish the marker array
        planar_regions_pub_.publish(marker_array);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher planar_regions_pub_;
    ros::Subscriber planar_terrain_sub_;

    std::vector<float> getColorByIndex(int index) {
        switch (index) {
            case 0: return {1.0, 0.0, 0.0}; // Red
            case 1: return {1.0, 0.5, 0.0}; // Orange
            case 2: return {1.0, 1.0, 0.0}; // Yellow
            case 3: return {0.0, 1.0, 0.0}; // Green
            case 4: return {0.0, 0.0, 1.0}; // Blue
            case 5: return {0.5, 0.0, 0.5}; // Purple
            case 6: return {1.0, 0.75, 0.8}; // Pink
            case 7: return {0.6, 0.3, 0.0}; // Brown
            default: return {0.5, 0.5, 0.5}; // Gray
        }
    }
};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "planar_region_republisher");

    // Create an instance of the PlanarRegionRepublisher class
    PlanarRegionRepublisher republisher;

    // Spin to keep the node running
    ros::spin();

    return 0;
}