#pragma once
#include <utility>
#include <vector>
#include "path_planner/grid_map.hpp"

// 优化版 A*（第二版）
//   - open set 用优先队列（最小堆），取 f 最小的节点只要 O(log n)
//   - g 值、父节点、是否已关闭都存在按格子编号索引的数组里，查找 O(1)
//   - 支持 8 邻域（可以斜着走），启发函数用 octile 距离；
//     斜着走时禁止"穿过障碍物的拐角"
//
// 返回从起点到终点的格子序列（包含起点和终点）；找不到路径时返回空。
// 起点允许在膨胀区里（小车可能被推进去了），但终点必须可通行。
std::vector<std::pair<int, int>> astarSearch(const GridMap& map,
                                             int start_row, int start_col,
                                             int goal_row, int goal_col,
                                             bool allow_diagonal = true);

// 启发函数（单位：格子）
double manhattanHeuristic(int row1, int col1, int row2, int col2);  // 4 邻域用
double octileHeuristic(int row1, int col1, int row2, int col2);     // 8 邻域用

// 路径长度（单位：格子；直走一格 = 1，斜走一格 = √2）
double pathLength(const std::vector<std::pair<int, int>>& path);

// 只保留路径的起点、拐点和终点，去掉直线中间的点
std::vector<std::pair<int, int>> simplifyPath(const std::vector<std::pair<int, int>>& path);

// 两个格子中心之间的直线是否全程可通行（精确检查直线经过的每一个格子）
bool hasLineOfSight(const GridMap& map, std::pair<int, int> from, std::pair<int, int> to);

// 路径拉直：如果从某个点能直线"看到"更后面的点（中间没有障碍物），
// 就跳过中间的点。得到拐点更少、方向不再限于 45° 倍数的路径。
std::vector<std::pair<int, int>> smoothPath(const GridMap& map,
                                            const std::vector<std::pair<int, int>>& path);
