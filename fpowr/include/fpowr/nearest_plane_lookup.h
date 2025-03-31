#ifndef FPOWR_NEAREST_PLANE_LOOKUP_H_
#define FPOWR_NEAREST_PLANE_LOOKUP_H_ 

#include <boost/geometry.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <tf/transform_datatypes.h>

namespace bg = boost::geometry;

using point_t = bg::model::d2::point_xy<double>;
using polygon_t = bg::model::polygon<point_t>;

namespace fpowr {

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
                // Convert local point to world point
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
        
            // Convert position to point
            point_t point(position(0), position(1));
        
            // Find the nearest polygon
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

} // namespace fpowr

#endif /* FPOWR_NEAREST_PLANE_LOOKUP_H_ */