#include "path_planner/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>

namespace
{

const double kSqrt2 = std::sqrt(2.0);

// 8 个方向：前 4 个是上下左右（代价 1），后 4 个是斜向（代价 √2）
const int kDirRow[8] = {1, -1, 0, 0, 1, 1, -1, -1};
const int kDirCol[8] = {0, 0, -1, 1, -1, 1, -1, 1};

// 放进优先队列的元素
struct OpenItem
{
    double f;    // f = g + h，越小越优先
    double h;    // f 相同时，h 小的（离终点更近的）优先，可以少展开一些节点
    int index;   // 格子编号 = row * width + col
};

// std::priority_queue 默认是"最大堆"（最大的在顶上）。
// 这个比较器反过来定义"谁更小"，让 f 最小的元素排在顶上，变成最小堆。
struct OpenItemGreater
{
    bool operator()(const OpenItem& a, const OpenItem& b) const
    {
        if (a.f != b.f)
        {
            return a.f > b.f;
        }
        return a.h > b.h;
    }
};

}  // namespace

double manhattanHeuristic(int row1, int col1, int row2, int col2)
{
    return std::abs(row1 - row2) + std::abs(col1 - col2);
}

double octileHeuristic(int row1, int col1, int row2, int col2)
{
    // 只能直走或斜走（45°）时，两点间的最短距离：
    // 先斜着走 min(dx, dy) 步，再直着走剩下的 |dx - dy| 步
    int dx = std::abs(col1 - col2);
    int dy = std::abs(row1 - row2);
    return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
}

std::vector<std::pair<int, int>> astarSearch(const GridMap& map,
                                             int start_row, int start_col,
                                             int goal_row, int goal_col,
                                             bool allow_diagonal)
{
    if (!map.isInside(start_row, start_col) || !map.isInside(goal_row, goal_col) ||
        map.isOccupied(goal_row, goal_col))
    {
        return {};
    }

    const int width = map.width();
    const int cell_count = width * map.height();
    const int start = start_row * width + start_col;
    const int goal = goal_row * width + goal_col;
    const int direction_count = allow_diagonal ? 8 : 4;

    auto heuristic = [&](int row, int col) {
        return allow_diagonal ? octileHeuristic(row, col, goal_row, goal_col)
                              : manhattanHeuristic(row, col, goal_row, goal_col);
    };

    // 每个格子一个位置，用格子编号直接访问，不需要再遍历查找
    std::vector<double> g_cost(cell_count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(cell_count, -1);
    std::vector<bool> closed(cell_count, false);

    std::priority_queue<OpenItem, std::vector<OpenItem>, OpenItemGreater> open_set;

    g_cost[start] = 0.0;
    double start_h = heuristic(start_row, start_col);
    open_set.push({start_h, start_h, start});

    while (!open_set.empty())
    {
        OpenItem current = open_set.top();
        open_set.pop();

        // 同一个格子可能被放进队列好几次（每次找到更短的路就放一次）。
        // 已经关闭的格子说明更好的那次已经处理过了，这条是过时的，直接跳过。
        // 这种做法叫"懒惰删除"，比在堆里查找并修改元素简单得多。
        if (closed[current.index])
        {
            continue;
        }
        closed[current.index] = true;

        if (current.index == goal)
        {
            // 顺着父节点从终点走回起点，再反转
            std::vector<std::pair<int, int>> path;
            for (int i = goal; i != -1; i = parent[i])
            {
                path.push_back({i / width, i % width});
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        int row = current.index / width;
        int col = current.index % width;

        for (int d = 0; d < direction_count; ++d)
        {
            int next_row = row + kDirRow[d];
            int next_col = col + kDirCol[d];

            if (!map.isInside(next_row, next_col) || map.isOccupied(next_row, next_col))
            {
                continue;
            }

            bool diagonal = (d >= 4);
            // 斜着走时，两侧相邻的格子都必须可通行，否则就是"贴着拐角斜穿"，小车会擦到障碍物
            if (diagonal && (map.isOccupied(row + kDirRow[d], col) ||
                             map.isOccupied(row, col + kDirCol[d])))
            {
                continue;
            }

            int next = next_row * width + next_col;
            if (closed[next])
            {
                continue;
            }

            double new_g = g_cost[current.index] + (diagonal ? kSqrt2 : 1.0);
            if (new_g < g_cost[next])
            {
                g_cost[next] = new_g;
                parent[next] = current.index;
                double h = heuristic(next_row, next_col);
                open_set.push({new_g + h, h, next});
            }
        }
    }

    return {};  // open set 空了还没到终点：没有路径
}

double pathLength(const std::vector<std::pair<int, int>>& path)
{
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        length += std::hypot(path[i].first - path[i - 1].first,
                             path[i].second - path[i - 1].second);
    }
    return length;
}

std::vector<std::pair<int, int>> simplifyPath(const std::vector<std::pair<int, int>>& path)
{
    if (path.size() <= 2)
    {
        return path;
    }

    std::vector<std::pair<int, int>> result;
    result.push_back(path.front());

    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        // 进入这个点的方向 和 离开这个点的方向
        int in_row = path[i].first - path[i - 1].first;
        int in_col = path[i].second - path[i - 1].second;
        int out_row = path[i + 1].first - path[i].first;
        int out_col = path[i + 1].second - path[i].second;

        // 方向变了，说明这是一个拐点，需要保留
        if (in_row != out_row || in_col != out_col)
        {
            result.push_back(path[i]);
        }
    }

    result.push_back(path.back());
    return result;
}

bool hasLineOfSight(const GridMap& map, std::pair<int, int> from, std::pair<int, int> to)
{
    // 精确的格子遍历（Amanatides-Woo 算法）：从起点格子出发，沿着线段
    // 每次跨过一条格子边界就进入下一个格子，保证线段经过的每个格子都被检查到。
    // （如果只是沿线每隔一小段采样，线段擦过格子一角时可能正好漏掉。）
    // 坐标约定：格子 (row, col) 覆盖 [row, row+1) x [col, col+1)，线段连接两个格子的中心。
    int row = from.first;
    int col = from.second;
    const int d_row = to.first - from.first;
    const int d_col = to.second - from.second;
    const int step_row = (d_row > 0) ? 1 : -1;
    const int step_col = (d_col > 0) ? 1 : -1;
    const double inf = std::numeric_limits<double>::infinity();

    // t 表示沿线段走过的比例（0 = 起点，1 = 终点）。
    // 起点在格子中心，离下一条边界 0.5 格；之后每跨一格 t 增加 1 / |d|。
    const double t_delta_row = (d_row != 0) ? 1.0 / std::abs(d_row) : inf;
    const double t_delta_col = (d_col != 0) ? 1.0 / std::abs(d_col) : inf;
    double t_next_row = 0.5 * t_delta_row;  // 下一次跨过横向边界（row 变化）时的 t
    double t_next_col = 0.5 * t_delta_col;  // 下一次跨过纵向边界（col 变化）时的 t
    const double eps = 1e-9;

    if (map.isOccupied(row, col))
    {
        return false;
    }
    const int max_steps = std::abs(d_row) + std::abs(d_col);
    for (int i = 0; i < max_steps && (row != to.first || col != to.second); ++i)
    {
        if (t_next_row < t_next_col - eps)
        {
            row += step_row;
            t_next_row += t_delta_row;
        }
        else if (t_next_col < t_next_row - eps)
        {
            col += step_col;
            t_next_col += t_delta_col;
        }
        else
        {
            // 线段正好穿过格子的角点：两侧的格子都必须可通行（和 A* 禁止斜穿拐角一致）
            if (map.isOccupied(row + step_row, col) || map.isOccupied(row, col + step_col))
            {
                return false;
            }
            row += step_row;
            col += step_col;
            t_next_row += t_delta_row;
            t_next_col += t_delta_col;
        }

        if (map.isOccupied(row, col))
        {
            return false;
        }
    }
    return true;
}

std::vector<std::pair<int, int>> smoothPath(const GridMap& map,
                                            const std::vector<std::pair<int, int>>& path)
{
    if (path.size() <= 2)
    {
        return path;
    }

    std::vector<std::pair<int, int>> result;
    result.push_back(path.front());

    size_t anchor = 0;  // 当前保留的最后一个点
    while (anchor + 1 < path.size())
    {
        // 从最远的点往回找：第一个能被 anchor 直线看到的点，就是下一个保留点
        size_t next = anchor + 1;
        for (size_t j = path.size() - 1; j > anchor + 1; --j)
        {
            if (hasLineOfSight(map, path[anchor], path[j]))
            {
                next = j;
                break;
            }
        }
        result.push_back(path[next]);
        anchor = next;
    }
    return result;
}
