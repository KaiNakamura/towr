#ifndef GRID_HEIGHTMAP_H_
#define GRID_HEIGHTMAP_H_

#include "height_map.h"
#include <Eigen/Dense>

class Grid final : public towr::HeightMap
{

private:
    Eigen::MatrixXd grid_;
    const double res_m_p_cell_ = 0.17;
    const double eps_ = 0.03;

public:
    Grid()
    {
        // preform pre-processing on heightmap
        grid_ = Eigen::MatrixXd(2, 2);
        grid_ << 1, 2,
                 3, 4;
    };

    double GetHeight(double x, double y) const override
    {
        const size_t x_cell = static_cast<size_t>(x / res_m_p_cell_);
        const size_t y_cell = static_cast<size_t>(y / res_m_p_cell_);
        if ((x_cell >= grid_.cols()) || (y_cell >= grid_.rows()))
        {
            return 0.0;
        }
        return grid_(y_cell, x_cell);
    };

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