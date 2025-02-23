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
#include <geometry_msgs/Point.h>
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

#include <boost/geometry.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/distance.hpp>

namespace bg = boost::geometry;

using point_t = bg::model::d2::point_xy<double>;
using polygon_t = bg::model::polygon<point_t>;

namespace towr {

    std::vector<polygon_t> PlanarRegionsToPolygons(const convex_plane_decomposition_msgs::PlanarTerrain& terrain)
    {
        std::vector<polygon_t> polygons;

        for (const convex_plane_decomposition_msgs::PlanarRegion& region : terrain.planarRegions)
        {
            // Extract plane parameters
            tf::Quaternion q(region.plane_parameters.orientation.x,
                region.plane_parameters.orientation.y,
                region.plane_parameters.orientation.z,
                region.plane_parameters.orientation.w);
            tf::Matrix3x3 R(q);

            polygon_t poly;
            
            for (const auto& local_point : region.boundary.outer_boundary.points)
            {
                // convert local point to world point
                tf::Vector3 local_vec(local_point.x, local_point.y, 0.0);
                tf::Vector3 world_vec = R * local_vec + tf::Vector3(region.plane_parameters.position.x,
                                                                region.plane_parameters.position.y,
                                                                region.plane_parameters.position.z);
                bg::append(poly.outer(), point_t(world_vec.x(), world_vec.y()));
            }

            polygons.push_back(poly);
        }

        return polygons;
    }

    class NearestPlaneLookup
    {
    protected:
        // std::vector<convex_plane_decomposition_msgs::PlanarRegion> planarRegions_;
        std::vector<polygon_t> polygons_;

    public:
        NearestPlaneLookup(const convex_plane_decomposition_msgs::PlanarTerrain& terrain)
        {
            polygons_ = PlanarRegionsToPolygons(terrain);
        }

        int GetNearestPlaneIndex(const Eigen::VectorXd& position) const
        {
            // Ensure position has at least 2 elements
            assert(position.size() >= 2 && "Position vector must have at least 2 elements");
        
            // convert position to point
            point_t point(position(0), position(1));
        
            // find the nearest polygon
            double min_distance = std::numeric_limits<double>::max();
            int nearest_plane_index = -1;
            for (int i = 0; i < polygons_.size(); ++i)
            {
                double distance = bg::distance(point, polygons_[i]);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_plane_index = i;
                }
            }
        
            return nearest_plane_index;
        }
    };

} // namespace towr

#endif /* TOWR_ROS_NEAREST_PLANE_LOOKUP_H_ */