#ifndef TOWR_ROS_NEAREST_PLANE_LOOKUP_H_
#define TOWR_ROS_NEAREST_PLANE_LOOKUP_H_ 

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <towr_ros/FootstepPlanAction.h>
#include <towr/terrain/grid_height_map.h>
#include <towr/terrain/height_map_from_csv.h>
#include <towr/nlp_formulation.h>
#include <towr/initialization/gait_generator.h>
#include <towr/models/endeffector_mappings.h>
#include <xpp_states/convert.h>
#include <ifopt/ipopt_solver.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <set>

#include <cpptrace/from_current.hpp>
#include <boost/stacktrace.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <towr/variables/spline_holder.h>
#include <towr/nlp_formulation.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <tf/transform_datatypes.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

namespace towr {

using Point = std::pair<double, double>;
using Polygon = std::vector<Point>;

double CrossProduct(const Point& O, const Point& A, const Point& B) {
    return (A.first - O.first) * (B.second - O.second) - (A.second - O.second) * (B.first - O.first);
}

// Function to convert PlanarRegion to Polygon. Ignores any islands
Polygon ConvertPlanarRegionToPolygon(const convex_plane_decomposition_msgs::PlanarRegion& region) {
    Polygon polygon;

    // Extract plane parameters
    tf::Quaternion q(region.plane_parameters.orientation.x,
                        region.plane_parameters.orientation.y,
                        region.plane_parameters.orientation.z,
                        region.plane_parameters.orientation.w);
    tf::Matrix3x3 R(q);

    for (const auto& point : region.boundary.outer_boundary.points) {
        tf::Vector3 p_2d(point.x, point.y, 0.0);
        tf::Vector3 p_3d = R * p_2d + tf::Vector3(region.plane_parameters.position.x,
                                                    region.plane_parameters.position.y,
                                                    region.plane_parameters.position.z);
        polygon.push_back({p_3d.x(), p_3d.y()});
    }
    return polygon;
}

// Function to check if a point is inside a convex polygon
bool IsPointInConvexPolygon(const Point& point, const Polygon& polygon) {
    int n = polygon.size();
    if (n < 3) return false; // A polygon must have at least 3 vertices

    bool is_inside = true;
    for (int i = 0; i < n; ++i) {
        Point A = polygon[i];
        Point B = polygon[(i + 1) % n];
        if (CrossProduct(A, B, point) < 0) {
            is_inside = false;
            break;
        }
    }
    return is_inside;
}

// Custom comparator for Eigen::Vector2i
struct CompareEigenVector2i {
    bool operator()(const Eigen::Vector2i& a, const Eigen::Vector2i& b) const {
        return std::tie(a.x(), a.y()) < std::tie(b.x(), b.y());
    }
};

// Custom comparator for std::pair<double, Eigen::Vector2i>
struct ComparePair {
    bool operator()(const std::pair<double, Eigen::Vector2i>& a, const std::pair<double, Eigen::Vector2i>& b) const {
        return a.first > b.first;
    }
};

class NearestPlaneLookup
{
protected:
    Eigen::MatrixXd nearest_plane_map_;
    // store the terrain data
    grid_map::GridMap grid_map_;

public:
    NearestPlaneLookup(const convex_plane_decomposition_msgs::PlanarTerrain& terrain)
    {

        // Convert grid_map_msgs::GridMap to grid_map::GridMap
        grid_map::GridMap grid_map;
        grid_map::GridMapRosConverter::fromMessage(terrain.gridmap, grid_map);
        grid_map_ = grid_map;

        // set nearest_plane_map_ to the same shape as the height map, fill with -1s
        nearest_plane_map_ = Eigen::MatrixXd::Constant(grid_map.getLength().x(), grid_map.getLength().y(), -1);
        
        // find all grid cells that are within a polygon. set their value to the index of the polygon
        // switch this to a standard for loop so the index can be used
        // for (const auto& region : terrain.planarRegions) {
        for (int i = 0; i < terrain.planarRegions.size(); ++i) {
            Polygon polygon = ConvertPlanarRegionToPolygon(terrain.planarRegions[i]);
            for (int x = 0; x < nearest_plane_map_.rows(); ++x) {
                for (int y = 0; y < nearest_plane_map_.cols(); ++y) {
                    // map x, y to world coordinates
                    // does this work? I'm not sure
                    // TODO: make sure it is row major
                    const grid_map::Index index(x, y);
                    grid_map::Position position;
                    grid_map_.getPosition(index, position);
                    if (IsPointInConvexPolygon({position.x(), position.y()}, polygon)) {
                        nearest_plane_map_(x, y) = i;
                    }
                }
            }
        }

        // deep copy nearest_plane_map_
        Eigen::MatrixXd nearest_plane_map_copy = nearest_plane_map_;

        // GreedyBFS out from each cell marked with -1 to find the nearest cell with a value
        // use euclidean distance as the cost function
        // only check values in the plane map copy to not mess up search
        for (int x = 0; x < nearest_plane_map_.rows(); ++x) {
            for (int y = 0; y < nearest_plane_map_.cols(); ++y) {
                if (nearest_plane_map_(x, y) != -1) {break;}
                auto search_origin = Eigen::Vector2i(x, y);
                // GreedyBFS with euclidean cost function using a priority queue
                std::priority_queue<std::pair<double, Eigen::Vector2i>, std::vector<std::pair<double, Eigen::Vector2i>>, ComparePair> queue;
                // set of visited nodes
                std::set<Eigen::Vector2i, CompareEigenVector2i> visited;
                queue.push({0, Eigen::Vector2i(x, y)});
                while (!queue.empty()) {
                    auto [cost, index] = queue.top();
                    queue.pop();
                    if (visited.count(index) > 0) {continue;}
                    visited.insert(index);
                    if (nearest_plane_map_copy(index.x(), index.y()) != -1) {
                        nearest_plane_map_(x, y) = nearest_plane_map_copy(index.x(), index.y());
                        break;
                    }
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            if (dx == 0 && dy == 0) continue;
                            Eigen::Vector2i new_index = index + Eigen::Vector2i(dx, dy);
                            if (new_index.x() < 0 || new_index.x() >= nearest_plane_map_.rows() || new_index.y() < 0 || new_index.y() >= nearest_plane_map_.cols()) continue;
                            // Switch to euclidean distance from the search origin
                            double new_cost = (new_index - search_origin).norm();
                            queue.push({new_cost, new_index});
                        }
                    }
                }
            }
        }
    }

    int GetNearestPlaneIndex(const grid_map::Position position) const
    {
        // map position to grid coordinates
        grid_map::Index index;
        grid_map_.getIndex(position, index);
        return nearest_plane_map_(index.x(), index.y());
    }
};

} // namespace towr

#endif /* TOWR_ROS_NEAREST_PLANE_LOOKUP_H_ */