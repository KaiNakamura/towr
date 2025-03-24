#ifndef GRID_HEIGHT_MAP_H_
#define GRID_HEIGHT_MAP_H_

#include "height_map.h"
#include <Eigen/Dense>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <limits>

// #define GRID_HEIGHT_MAP_H_THROW_ON_OUT_OF_RANGE

class Grid final : public towr::HeightMap
{
private:
    grid_map::GridMap map_;
    double eps_; // value used to calculate the derivative

public:
    Grid(const convex_plane_decomposition_msgs::PlanarTerrain& terrain)
    {
        grid_map::GridMapRosConverter::fromMessage(terrain.gridmap, map_);
        eps_ = map_.getResolution() / 6.0;
        ROS_INFO("GridMap received");
    };

    double GetHeight(double x, double y) const {
        // Note that the height interpolation method here will change the behavior of the
        // derivative calculation
        grid_map::Position position(x, y);

        #ifdef GRID_HEIGHT_MAP_H_THROW_ON_OUT_OF_RANGE
            return map_.atPosition("elevation", position, grid_map::InterpolationMethods::INTER_LINEAR); 
        #endif

        float height;
        try{
            height = map_.atPosition("elevation", position, grid_map::InterpolationMethods::INTER_LINEAR); 
        } catch (const std::out_of_range& e) {
            height = std::numeric_limits<float>::max();
        }
        return height;
    }

    double GetHeightDerivWrtX(double x, double y) const override
    {
        float height_right = GetHeight(x + eps_, y);
        float height_left = GetHeight(x - eps_, y);
        return (height_right - height_left) / (2 * eps_);
    }

    double GetHeightDerivWrtY(double x, double y) const override
    {
        float height_up = GetHeight(x, y + eps_);
        float height_down = GetHeight(x, y - eps_);
        return (height_up - height_down) / (2 * eps_);
    }
};

#endif // GRID_HEIGHTMAP_H_