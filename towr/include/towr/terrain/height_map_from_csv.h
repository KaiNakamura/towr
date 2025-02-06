#ifndef HEIGHT_MAP_FROM_CSV_H_
#define HEIGHT_MAP_FROM_CSV_H_

#include <iostream>
#include "height_map.h"
#include <Eigen/Dense>
#include "rapidcsv.h"

/*
 * Implements the height map interface by using a 2D grid read from a CSV file,
 * with each element representing the environment height value.
 */
class HeightMapFromCSV final : public towr::HeightMap
{
public:
    HeightMapFromCSV(const std::string& file_path)
    {
        // open csv file and convert it to an eigen matrix
        rapidcsv::Document doc(file_path, rapidcsv::LabelParams(-1,-1));
        grid_ = Eigen::MatrixXd::Zero(doc.GetRowCount(), doc.GetColumnCount());
        for (size_t i{}; i<doc.GetRowCount(); ++i){
            for (size_t j{}; j<doc.GetColumnCount(); ++j){
                grid_(i, j) = doc.GetCell<double>(j, i);
            }
        }
    };

    // Return the terrain height at a particular location.
    double GetHeight(double x, double y) const override
    {
        const size_t x_cell = static_cast<size_t>(x/res_m_p_cell_);
        const size_t y_cell = static_cast<size_t>(y/res_m_p_cell_);
        if ((x_cell >= grid_.cols()) || (y_cell >= grid_.rows())){
            return 0.0;
        }
        return grid_(y_cell, x_cell);
    };

    // Return the approximate slope in the x direction at a particular location.
    double GetHeightDerivWrtX(double x, double y) const override
    {
        const size_t x_cell = static_cast<size_t>(x/res_m_p_cell_);
        const size_t y_cell = static_cast<size_t>(y/res_m_p_cell_);
        // check if current cell is valid
        const auto isCellValid = [this](const size_t x_c, const size_t y_c){
            return (x_c < grid_.cols()) && (y_c < grid_.rows());
        };
        if (!isCellValid(x_cell, y_cell)){
            return 0.0;
        }
        

        // if next cell higher, then use slope before it
        const size_t x_next_cell = x_cell + 1;
        if (isCellValid(x_next_cell, y_cell)){
            const double diff_end = grid_(y_cell, x_next_cell) - grid_(y_cell, x_cell);
            const double x_end = x_next_cell * res_m_p_cell_;
            if ((diff_end > 0) && (x <= x_end) && (x >= x_end - eps_)){
                return diff_end / eps_;
            }
        }

        // if previous cell is higher, then use slope after it
        const size_t x_prev_cell = x_cell - 1;
        if (isCellValid(x_prev_cell, y_cell)){
            const double diff_start = grid_(y_cell, x_cell) - grid_(y_cell, x_prev_cell);
            const double x_start = x_cell * res_m_p_cell_;
            if ((diff_start < 0) && (x >= x_start) && (x <= x_start + eps_)){
                return diff_start / eps_;
            }
        }
        
        return 0.0;
    }

    // Return the approximate slope in the y direction at a particular location.
    double GetHeightDerivWrtY(double x, double y) const override
    {
        const size_t x_cell = static_cast<size_t>(x/res_m_p_cell_);
        const size_t y_cell = static_cast<size_t>(y/res_m_p_cell_);
        // check if current cell is valid
        const auto isCellValid = [this](const size_t x_c, const size_t y_c){
            return (x_c < grid_.cols()) && (y_c < grid_.rows());
        };
        if (!isCellValid(x_cell, y_cell)){
            return 0.0;
        }
        

        // if next cell higher, then use slope before it
        const size_t y_next_cell = y_cell + 1;
        if (isCellValid(x_cell, y_next_cell)){
            const double diff_end = grid_(y_next_cell, x_cell) - grid_(y_cell, x_cell);
            const double y_end = y_next_cell * res_m_p_cell_;
            if ((diff_end > 0) && (y <= y_end) && (y >= y_end - eps_)){
                return diff_end / eps_;
            }
        }

        // if previous cell is higher, then use slope after it
        const size_t y_prev_cell = y_cell - 1;
        if (isCellValid(x_cell, y_prev_cell)){
            const double diff_start = grid_(y_cell, x_cell) - grid_(y_prev_cell, x_cell);
            const double y_start = y_cell * res_m_p_cell_;
            if ((diff_start < 0) && (y >= y_start) && (y <= y_start + eps_)){
                return diff_start / eps_;
            }
        }
        
        return 0.0;
    }

  private:
    Eigen::MatrixXd grid_;
    const double res_m_p_cell_ = 0.17;
    const double eps_ = res_m_p_cell_ / 50;
  };

#endif