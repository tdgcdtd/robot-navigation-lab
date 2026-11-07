#pragma once
#include <vector>
#include <cstdint>

class GridMap
{
public:
    // 格子的取值，和 ROS 的 nav_msgs/OccupancyGrid 约定一致（0 = 空闲，100 = 障碍物）
    static constexpr int8_t kFree = 0;
    static constexpr int8_t kInflated = 50;   // 膨胀区：本身没有障碍物，但小车中心不能进入
    static constexpr int8_t kOccupied = 100;

    GridMap(int width, int height, double resolution,
            double origin_x, double origin_y);

    bool isInside(int row, int col) const;
    void setOccupied(int row, int col);
    // 是否不可通行（障碍物或膨胀区）
    bool isOccupied(int row, int col) const;
    int8_t getCost(int row, int col) const;
    bool worldToGrid(double x, double y, int& row, int& col) const;
    void gridToWorld(int row, int col, double& x, double& y) const;
    void setOccupiedRect(double min_x, double min_y,double max_x, double max_y);
    // 障碍物膨胀：把离障碍物 radius 米以内的格子也标为障碍物
    void inflate(double radius);
    void print() const;

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }

private:
    int width_;
    int height_;

    double resolution_;
    double origin_x_;
    double origin_y_;

    std::vector<int8_t> data_;

    int index(int row, int col) const;
};