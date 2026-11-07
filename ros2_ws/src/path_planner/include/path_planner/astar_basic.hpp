#pragma once
// 第一版（教学版）A*，只用于性能对比。新代码请用 astar_planner.hpp 里的 astarSearch。
#include <vector>
#include <utility>
#include "path_planner/grid_map.hpp"

struct GridNode
{
    int row;
    int col;

    double g_cost;
    double h_cost;

    int parent_row;
    int parent_col;

    double f_cost() const
    {
        return g_cost + h_cost;
    }
};

double manhattanDistance(int row1, int col1,int row2, int col2);
std::vector<std::pair<int, int>> getNeighbors(const GridMap& map, int row, int col);
int findLowestFCostIndex(const std::vector<GridNode>& open_set);
std::vector<std::pair<int, int>> astarSearchBasic(const GridMap& map,int start_row, int start_col,int goal_row, int goal_col);
std::vector<std::pair<int, int>> reconstructPath(const std::vector<GridNode>& closed_set,const GridNode& goal_node);
