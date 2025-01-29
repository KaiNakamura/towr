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
    Eigen::MatrixXd grid_;
    grid_map_msgs::GridMap gridmap_;
    const double res_m_p_cell_ = 0.17;
    const double eps_ = 0.03;

public:
    Grid(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& terrain)
    {
        gridmap_ = terrain->gridmap;
        ROS_INFO("GridMap received");
    };

    double GetHeight(double x, double y) const {
        grid_map::GridMap map;
        grid_map::GridMapRosConverter::fromMessage(gridmap_, map);

        grid_map::Position position(x, y);
        if (map.isInside(position)) {
            return map.atPosition("elevation", position);
        } else {
            throw std::out_of_range("Coordinates are outside the grid map.");
        }
    }

    double GetHeightDerivWrtX(double x, double y) const override
    {
        const size_t x_cell = static_cast<size_t>(x / res_m_p_cell_);
        const size_t y_cell = static_cast<size_t>(y / res_m_p_cell_);
        if ((x_cell + 1 >= grid_.cols()) || (y_cell >= grid_.rows()))
        {
            return 0.0;
        }

        const double diff = grid_(y_cell, x_cell + 1) - grid_(y_cell, x_cell);

        const double x_start_cell = (x_cell + 1) * res_m_p_cell_;
        if ((x <= (x_start_cell + eps_ / 2)) && (x >= x_start_cell - eps_ / 2))
        {
            return diff / eps_;
        }
        return 0.0;
    }

    double GetHeightDerivWrtY(double x, double y) const override
    {
        const size_t x_cell = static_cast<size_t>(x / res_m_p_cell_);
        const size_t y_cell = static_cast<size_t>(y / res_m_p_cell_);
        if ((x_cell >= grid_.cols()) || (y_cell + 1 >= grid_.rows()))
        {
            return 0.0;
        }

        const double diff = grid_(y_cell + 1, x_cell) - grid_(y_cell, x_cell);

        const double y_start_cell = (y_cell + 1) * res_m_p_cell_;
        if ((y <= (y_start_cell + eps_ / 2)) && (y >= y_start_cell - eps_ / 2))
        {
            return diff / eps_;
        }
        return 0.0;
    }
};

#endif // GRID_HEIGHTMAP_H_