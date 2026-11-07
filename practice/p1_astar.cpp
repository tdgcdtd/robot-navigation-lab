// P1 核心手写 ②：A*（4 邻域、曼哈顿启发、优先队列 + 懒惰删除）
// 编译运行：
//   cd ~/project/robot-navigation-lab/practice
//   g++ -std=c++17 p1_astar.cpp -o p1_astar && ./p1_astar
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <queue>
#include <random>
#include <utility>
#include <vector>

// ---------------- 简化版栅格地图（不用改） ----------------
struct Grid
{
    int width;
    int height;
    std::vector<bool> blocked;  // blocked[row * width + col] == true 表示障碍物

    Grid(int w, int h) : width(w), height(h), blocked(w * h, false) {}

    bool isInside(int row, int col) const { return row >= 0 && row < height && col >= 0 && col < width; }
    bool isFree(int row, int col) const { return isInside(row, col) && !blocked[row * width + col]; }
    void block(int row, int col) { blocked[row * width + col] = true; }
};

using Cell = std::pair<int, int>;  // (row, col)
using Path = std::vector<Cell>;

double manhattan(Cell a, Cell b)
{
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

// 找从 start 到 goal 的最短路径（只能上下左右走，每步代价 1）。
// 返回：包含起点和终点的格子序列；没有路径时返回空 vector。
Path astar(const Grid& grid, Cell start, Cell goal)
{
    // ========== 在这里写你的代码 ==========
    //
    // 步骤提示（按顺序来，每一步都不长）：
    //  0. 特殊情况：起点或终点不可通行 -> 返回空
    if(!grid.isFree(start.first, start.second) || !grid.isFree(goal.first, goal.second))
    {
        return {};
    }
    //  1. 准备三个数组（长度 = 格子总数，用"格子编号 = row * width + col"访问）：
    //       g_cost（初始为无穷大）、parent（初始为 -1）、closed（初始为 false）
    const int total_cells = grid.width * grid.height;
    std::vector<double> g_cost(total_cells, std::numeric_limits<double>::infinity());
    std::vector<int> parent(total_cells, -1);
    std::vector<bool> closed(total_cells, false);

    //  2. 准备优先队列（最小堆），里面放 (f, 格子编号)，f 小的先出来
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> open_set;
    //  3. 起点：g = 0，放进队列
    int start_id = start.first * grid.width + start.second;
    int goal_id = goal.first * grid.width + goal.second;
    g_cost[start_id] = 0.0;
    open_set.push({0.0, start_id});
    //  4. 循环：队列不空时
    //       a. 取出 f 最小的，弹出
    //       b. 已经 closed 过 -> 跳过（懒惰删除）；否则标记 closed
    //       c. 是终点 -> 顺着 parent 回溯出路径，反转后返回
    //       d. 看上下左右 4 个邻居：不可通行或已 closed 就跳过；
    //          new_g = 当前 g + 1；如果 new_g 比邻居现在的 g 小：
    //          更新 g、parent，算 h（曼哈顿距离），把 (new_g + h, 邻居编号) 放进队列
    const int dr[4] = {1, -1, 0, 0};
    const int dc[4] = {0, 0, 1, -1};
    
    while(!open_set.empty())
    {
        std::pair<double, int> current = open_set.top();
        open_set.pop();

        int current_id = current.second;
        if(closed[current_id])
            continue; // 懒惰删除
        closed[current_id] = true;

        if(current_id == goal_id)
        {
            // 回溯路径
            Path path;
            for(int id = goal_id; id != -1; id = parent[id])
            {
                int row = id / grid.width;
                int col = id % grid.width;
                path.push_back({row, col});
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        int current_row = current_id / grid.width;
        int current_col = current_id % grid.width;

        for(int d = 0; d < 4; d++)
        {
            int nr = current_row + dr[d];
            int nc = current_col + dc[d];
            int neighbor_id = nr * grid.width + nc;
            Cell neighbor = {nr, nc};

            if(!grid.isFree(nr, nc) || closed[neighbor_id])
                continue;
            double new_g = g_cost[current_id] + 1;
            if (new_g < g_cost[neighbor_id])
            {
                g_cost[neighbor_id] = new_g;
                parent[neighbor_id] = current_id;
                double h = manhattan(neighbor, goal);
                open_set.push({h + g_cost[neighbor_id], neighbor_id});
            }
            
        }
    }
    //  5. 循环结束还没找到 -> 返回空

    return {};
}

// ---------------- 下面是测试，不用改 ----------------
int failures = 0;

// 广度优先搜索：4 邻域、每步代价 1 时，它找到的步数一定最短（用来对答案）
int bfsSteps(const Grid& grid, Cell start, Cell goal)
{
    if (!grid.isFree(start.first, start.second) || !grid.isFree(goal.first, goal.second)) return -1;
    std::vector<int> dist(grid.width * grid.height, -1);
    std::queue<Cell> q;
    q.push(start);
    dist[start.first * grid.width + start.second] = 0;
    const int dr[4] = {1, -1, 0, 0}, dc[4] = {0, 0, 1, -1};
    while (!q.empty())
    {
        Cell c = q.front(); q.pop();
        if (c == goal) return dist[c.first * grid.width + c.second];
        for (int d = 0; d < 4; ++d)
        {
            int r = c.first + dr[d], cc = c.second + dc[d];
            if (grid.isFree(r, cc) && dist[r * grid.width + cc] < 0)
            {
                dist[r * grid.width + cc] = dist[c.first * grid.width + c.second] + 1;
                q.push({r, cc});
            }
        }
    }
    return -1;
}

// 检查：首尾正确、每步只走到上下左右相邻的格子、不经过障碍物
bool validPath(const Grid& grid, const Path& path, Cell start, Cell goal)
{
    if (path.empty() || path.front() != start || path.back() != goal) return false;
    for (size_t i = 0; i < path.size(); ++i)
    {
        if (!grid.isFree(path[i].first, path[i].second)) return false;
        if (i > 0 && std::abs(path[i].first - path[i - 1].first) + std::abs(path[i].second - path[i - 1].second) != 1)
            return false;
    }
    return true;
}

void report(const char* name, bool ok, const char* detail)
{
    if (!ok) ++failures;
    std::printf("%s %-26s %s\n", ok ? "✅" : "❌", name, detail);
}

int main()
{
    char buf[128];
    {   // 1. 空地图
        Grid g(5, 5);
        Path p = astar(g, {0, 0}, {4, 4});
        bool ok = validPath(g, p, {0, 0}, {4, 4}) && p.size() == 9;
        std::snprintf(buf, sizeof(buf), "路径格子数=%zu（期望 9：走 8 步）", p.size());
        report("1 空地图 5x5", ok, buf);
    }
    {   // 2. 绕墙：第 2 列是墙，只在最上面一行留缺口
        Grid g(5, 5);
        for (int r = 0; r < 4; ++r) g.block(r, 2);
        Path p = astar(g, {0, 0}, {0, 4});
        bool ok = validPath(g, p, {0, 0}, {0, 4}) && (int)p.size() - 1 == bfsSteps(g, {0, 0}, {0, 4});
        std::snprintf(buf, sizeof(buf), "步数=%d（期望 %d）", (int)p.size() - 1, bfsSteps(g, {0, 0}, {0, 4}));
        report("2 绕墙", ok, buf);
    }
    {   // 3. 终点被围住
        Grid g(5, 5);
        g.block(1, 2); g.block(3, 2); g.block(2, 1); g.block(2, 3);
        Path p = astar(g, {0, 0}, {2, 2});
        report("3 终点被围住 -> 无路径", p.empty(), p.empty() ? "返回空" : "应该返回空");
    }
    {   // 4. 起点 = 终点
        Grid g(5, 5);
        Path p = astar(g, {2, 2}, {2, 2});
        bool ok = p.size() == 1 && p[0] == Cell(2, 2);
        report("4 起点 = 终点", ok, ok ? "只有 1 个格子" : "应该只包含起点这 1 个格子");
    }
    {   // 5. 终点是障碍物
        Grid g(5, 5);
        g.block(4, 4);
        Path p = astar(g, {0, 0}, {4, 4});
        report("5 终点是障碍物 -> 无路径", p.empty(), p.empty() ? "返回空" : "应该返回空");
    }
    {   // 6. 30 张随机地图，和 BFS 比最短步数
        int bad = 0;
        for (unsigned seed = 1; seed <= 30; ++seed)
        {
            Grid g(20, 20);
            std::mt19937 rng(seed);
            for (int r = 0; r < 20; ++r)
                for (int c = 0; c < 20; ++c)
                    if (rng() % 100 < 30) g.block(r, c);
            g.blocked[0] = false;
            g.blocked[20 * 20 - 1] = false;
            Path p = astar(g, {0, 0}, {19, 19});
            int expect = bfsSteps(g, {0, 0}, {19, 19});
            bool ok = (expect < 0) ? p.empty() : (validPath(g, p, {0, 0}, {19, 19}) && (int)p.size() - 1 == expect);
            if (!ok) ++bad;
        }
        std::snprintf(buf, sizeof(buf), "%d/30 张地图正确", 30 - bad);
        report("6 随机地图 vs 标准答案", bad == 0, buf);
    }
    std::printf(failures == 0 ? "\n🎉 全部通过！\n" : "\n还有 %d 个没通过，继续加油\n", failures);
    return failures == 0 ? 0 : 1;
}
