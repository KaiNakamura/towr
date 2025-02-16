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

#include <cpptrace/from_current.hpp>
#include <boost/stacktrace.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <towr/variables/spline_holder.h>
#include <towr/nlp_formulation.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <tf/transform_datatypes.h>

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

class NearestPlaneLookup
{
protected:
    Eigen::MatrixXd nearest_plane_map_;
    // store the terrain data
    convex_plane_decomposition::PlanarTerrain terrain_;

public:
    NearestPlaneLookup(const convex_plane_decomposition::PlanarTerrain& terrain)
    {
        terrain_ = terrain;
        // set nearest_plane_map_ to the same shape as the height map, fill with -1s
        nearest_plane_map_ = Eigen::MatrixXd::Constant(terrain.gridMap.getLength().x(), terrain.gridMap.getLength().y(), -1);
        
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
                    auto index = Eigen::Vector2d(x, y);
                    Eigen::Vector3d position;
                    terrain.gridMap.getPosition(index, position);
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
                std::priority_queue<std::pair<double, Eigen::Vector2i>> queue;
                // set of visited nodes
                auto visited = std::set<Eigen::Vector2i>();
                queue.push({0, Eigen::Vector2i(x, y)});
                while (!queue.empty()) {
                    auto [cost, index] = queue.top();
                    queue.pop();
                    if (visited.count(index) > 0) {continue;}
                    if (nearest_plane_map_copy(index.x(), index.y()) != -1) {
                        nearest_plane_map_(x, y) = nearest_plane_map_copy(index.x(), index.y());
                        break;
                    }
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            if (dx == 0 && dy == 0) continue;
                            Eigen::Vector2i new_index = index + Eigen::Vector2i(dx, dy);
                            if (new_index.x() < 0 || new_index.x() >= nearest_plane_map_.rows() || new_index.y() < 0 || new_index.y() >= nearest_plane_map_.cols()) continue;
                            // Switch to eucidlean distance from the search origin
                            double new_cost = (new_index - search_origin).norm();
                            queue.push({new_cost, new_index});
                        }
                    }
                }
            }
        }
    }

    int GetNearestPlaneIndex(const Eigen::Vector3d& position) const
    {
        // map position to grid coordinates
        Eigen::Vector2d index;
        terrain_.getPosition(index, position);
        return nearest_plane_map_(index.x(), index.y());
    }
}