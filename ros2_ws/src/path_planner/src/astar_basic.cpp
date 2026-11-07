// 第一版（教学版）A*：open set / closed set 都用 vector，每一步线性查找。
// 保留下来做性能对比（benchmark_astar），以及记录学习过程。新代码请用 astar_planner.hpp。
#include "path_planner/astar_basic.hpp"
#include "path_planner/grid_map.hpp"
#include <cmath>
#include <algorithm>

double manhattanDistance(int row1, int col1,
                         int row2, int col2)
{
    return std::abs(row1 - row2) + std::abs(col1 - col2);
}

int directions[4][2] = {
    {1, 0},  // Up
    {-1, 0}, // Down
    {0, -1}, // Left
    {0, 1}   // Right
};

std::vector<std::pair<int, int>> getNeighbors(const GridMap &map, int row, int col)
{
    std::vector<std::pair<int, int>> neighbors;

    for (int i = 0; i < 4; i++)
    {
        int new_row = row + directions[i][0];
        int new_col = col + directions[i][1];

        if (map.isInside(new_row, new_col) && !map.isOccupied(new_row, new_col))
        {
            neighbors.push_back({new_row, new_col}); // neighbors.push_back(std::make_pair(new_row, new_col));
        }
    }
    return neighbors;
}

int findLowestFCostIndex(const std::vector<GridNode> &open_set)
{
    int min_index = 0;

    for (int i = 1; i < static_cast<int>(open_set.size()); i++)
    {
        if (open_set[min_index].f_cost() > open_set[i].f_cost())
        {
            min_index = i;
        }
    }

    return min_index;
}

std::vector<std::pair<int, int>> reconstructPath(const std::vector<GridNode> &closed_set, const GridNode &goal_node)
{
    std::vector<std::pair<int, int>> path;
    GridNode current_node = goal_node;

    path.push_back({goal_node.row, goal_node.col});

    while (current_node.parent_row != -1 && current_node.parent_col != -1)
    {
        bool parent_found = false;

        for (int i = 0; i < static_cast<int>(closed_set.size()); i++)
        {
            if (current_node.parent_row == closed_set[i].row && current_node.parent_col == closed_set[i].col)
            {
                current_node = closed_set[i];
                path.push_back({closed_set[i].row, closed_set[i].col});
                parent_found = true;

                break;
            }
        }

        if (parent_found == false)
        {
            return {};
        }
    }

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<std::pair<int, int>> astarSearchBasic(const GridMap &map, int start_row, int start_col, int goal_row, int goal_col)
{
    std::vector<GridNode> open_set;
    std::vector<GridNode> closed_set;
    GridNode start_node;

    start_node.row = start_row;
    start_node.col = start_col;
    start_node.g_cost = 0;
    start_node.h_cost = manhattanDistance(start_row, start_col, goal_row, goal_col);
    start_node.parent_row = -1;
    start_node.parent_col = -1;

    open_set.push_back(start_node);

    while (!open_set.empty())
    {
        int current_index = findLowestFCostIndex(open_set);
        GridNode current_node = open_set[current_index];

        open_set.erase(open_set.begin() + current_index);
        closed_set.push_back(current_node);
        if (current_node.row == goal_row && current_node.col == goal_col)
        {
            return reconstructPath(closed_set, current_node);
        }

        auto neighbors = getNeighbors(
            map, current_node.row, current_node.col);

        for (const auto &neighbor : neighbors)
        {
            int neighbor_row = neighbor.first;
            int neighbor_col = neighbor.second;
            // 1. 判断当前邻居是否存在于 closed_set
            bool in_closed_set = false;
            
            for(int i = 0; i < static_cast<int>(closed_set.size()); i++)
            {
                if(closed_set[i].row == neighbor_row && closed_set[i].col == neighbor_col)
                {
                    in_closed_set = true; // 说明当前邻居已经存在于 closed_set 中
                    break; 
                }
            }
            // 2. 如果邻居节点已经在 closed_set 中，则跳过该邻居节点
            if(in_closed_set)
            {
                continue;
            }
            //  如果邻居节点不在 closed_set 中
            double tentative_g = current_node.g_cost + 1.0; // Assuming cost between adjacent nodes is 1
            //  检查是否已经在 open_set 中
            int open_index = -1;
            
            for(int i = 0; i < static_cast<int>(open_set.size()); i++)
            {
                if(neighbor_row == open_set[i].row && neighbor_col == open_set[i].col)
                {
                    open_index = i;
                    break;
                }
            }

            if(open_index == -1)
            {
                //  如果邻居节点不在 open_set 中，则将其添加到 open_set
                GridNode neighbor_node;
                neighbor_node.row = neighbor_row;
                neighbor_node.col = neighbor_col;
                neighbor_node.g_cost = tentative_g;
                neighbor_node.h_cost = manhattanDistance(neighbor_row, neighbor_col, goal_row, goal_col);
                neighbor_node.parent_row = current_node.row;
                neighbor_node.parent_col = current_node.col;

                open_set.push_back(neighbor_node);
            }
            else
            {
                //  如果邻居节点已经在 open_set 中，则检查是否需要更新 g_cost
                if(tentative_g < open_set[open_index].g_cost)
                {
                    open_set[open_index].g_cost = tentative_g;
                    open_set[open_index].parent_row = current_node.row;
                    open_set[open_index].parent_col = current_node.col;
                }
            }
        }
    }

    return {}; // Return empty path if no solution is found
}
