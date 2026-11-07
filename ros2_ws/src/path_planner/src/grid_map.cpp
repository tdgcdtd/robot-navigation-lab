#include "path_planner/grid_map.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

GridMap::GridMap(int width, int height, double resolution, double origin_x, double origin_y)
    : width_(width), height_(height), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y), data_(width * height, 0)
{
}

bool GridMap::isInside(int row, int col) const
{
    return row >= 0 && row < height_ && col >= 0 && col < width_;
}

void GridMap::setOccupied(int row, int col)
{
    if (isInside(row, col))
    {
        data_[index(row, col)] = kOccupied;
    }
}

bool GridMap::isOccupied(int row, int col) const
{
    if (isInside(row, col))
    {
        return data_[index(row, col)] != kFree;
    }
    return false;
}

int8_t GridMap::getCost(int row, int col) const
{
    if (isInside(row, col))
    {
        return data_[index(row, col)];
    }
    return kOccupied;  // 地图外面当作障碍物
}

int GridMap::index(int row, int col) const
{
    return row * width_ + col;
}

bool GridMap::worldToGrid(double x, double y, int& row, int& col) const
{
    col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    row = static_cast<int>(std::floor((y - origin_y_) / resolution_));

    return isInside(row, col);
}

void GridMap::gridToWorld(int row, int col, double& x, double& y) const
{
    x = origin_x_ + (col + 0.5) * resolution_;
    y = origin_y_ + (row + 0.5) * resolution_;
}

void GridMap::setOccupiedRect(double min_x, double min_y, double max_x, double max_y)
{
    int min_row = static_cast<int>(std::floor((min_y - origin_y_) / resolution_));
    int min_col = static_cast<int>(std::floor((min_x - origin_x_) / resolution_));

    int max_row = static_cast<int>(std::ceil((max_y - origin_y_) / resolution_) - 1);
    int max_col = static_cast<int>(std::ceil((max_x - origin_x_) / resolution_) - 1);

    for (int row = min_row; row <= max_row; ++row)
    {
        for (int col = min_col; col <= max_col; ++col)
        {
            setOccupied(row, col);
        }
    }
}

void GridMap::inflate(double radius)
{
    // 先复制一份原始地图：只从"原始障碍物"向外膨胀，
    // 否则新标记的格子会被当成障碍物再次膨胀，越扩越大。
    const std::vector<int8_t> original = data_;
    const int cell_radius = static_cast<int>(std::ceil(radius / resolution_));

    for (int row = 0; row < height_; ++row)
    {
        for (int col = 0; col < width_; ++col)
        {
            if (original[index(row, col)] != kOccupied)
            {
                continue;
            }

            for (int dr = -cell_radius; dr <= cell_radius; ++dr)
            {
                for (int dc = -cell_radius; dc <= cell_radius; ++dc)
                {
                    // 邻居格子中心到障碍物格子"方块"的最近距离。
                    // 减去 0.5 是因为障碍物格子本身有半个格子的宽度。
                    double gap_row = std::max(0.0, std::abs(dr) - 0.5);
                    double gap_col = std::max(0.0, std::abs(dc) - 0.5);
                    double distance = std::hypot(gap_row, gap_col) * resolution_;

                    int r = row + dr;
                    int c = col + dc;
                    // 只把空闲格子标为膨胀区，不覆盖真正的障碍物
                    if (distance <= radius && isInside(r, c) && data_[index(r, c)] == kFree)
                    {
                        data_[index(r, c)] = kInflated;
                    }
                }
            }
        }
    }
}

void GridMap::print() const
{
    for (int row = height_-1; row >= 0; --row)
    {
        for (int col = 0; col < width_; ++col)
        {
            if (getCost(row, col) == kOccupied)
            {
                std::cout << "# ";
            }
            else if (getCost(row, col) == kInflated)
            {
                std::cout << "+ ";
            }
            else
            {
                std::cout << ". ";
            }
        }
        std::cout << '\n';
    }
    
}
