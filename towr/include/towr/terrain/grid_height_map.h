#ifndef GRID_HEIGHT_MAP_H_
#define GRID_HEIGHT_MAP_H_

#include "height_map.h"
#include <Eigen/Dense>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>

class Grid final : public towr::HeightMap
{
private:
    grid_map::GridMap map_;
    double res_m_p_cell_;
    double eps_;

public:
    Grid(const convex_plane_decomposition_msgs::PlanarTerrain& terrain)
    {
        grid_map::GridMapRosConverter::fromMessage(terrain.gridmap, map_);
        res_m_p_cell_ = map_.getResolution();
        eps_ = res_m_p_cell_ * 0.176; // Assuming eps_ is a fraction of the resolution
        ROS_INFO("GridMap received");
    };

    double GetHeight(double x, double y) const {
        grid_map::Position position(x, y);
        if (map_.isInside(position)) {
            return map_.atPosition("elevation", position);
        } else {
            throw std::out_of_range("Coordinates are outside the grid map.");
        }
    }

    double GetHeightDerivWrtX(double x, double y) const override
    {
        grid_map::Position position(x, y);
        if (!map_.isInside(position)) {
            return 0.0;
        }

        grid_map::Position position_right(x + res_m_p_cell_, y);
        if (!map_.isInside(position_right)) {
            return 0.0;
        }

        const double diff = map_.atPosition("elevation", position_right) - map_.atPosition("elevation", position);

        if ((x <= (position_right.x() + eps_ / 2)) && (x >= position_right.x() - eps_ / 2)) {
            return diff / eps_;
        }
        return 0.0;
    }

    double GetHeightDerivWrtY(double x, double y) const override
    {
        grid_map::Position position(x, y);
        if (!map_.isInside(position)) {
            return 0.0;
        }

        grid_map::Position position_up(x, y + res_m_p_cell_);
        if (!map_.isInside(position_up)) {
            return 0.0;
        }

        const double diff = map_.atPosition("elevation", position_up) - map_.atPosition("elevation", position);

        if ((y <= (position_up.y() + eps_ / 2)) && (y >= position_up.y() - eps_ / 2)) {
            return diff / eps_;
        }
        return 0.0;
    }
};

#endif // GRID_HEIGHTMAP_H_