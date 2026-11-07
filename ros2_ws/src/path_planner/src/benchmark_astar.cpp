// A* 性能对比：第一版（vector + 线性查找）vs 优化版（优先队列 + 数组索引）
// 运行：ros2 run path_planner benchmark_astar
#include "path_planner/astar_basic.hpp"
#include "path_planner/astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

using Path = std::vector<std::pair<int, int>>;

struct Scenario
{
    std::string name;
    GridMap map;
    int start_row, start_col, goal_row, goal_col;
};

// nav_world 场景（障碍物和 robot_sim/config/nav_world.yaml 相同），起点 (0,0) -> 终点 (4,4)
Scenario makeNavWorld(double resolution)
{
    int cells = static_cast<int>(10.0 / resolution + 0.5);
    GridMap map(cells, cells, resolution, -5.0, -5.0);
    const double boxes[][4] = {{2.0, 1.0, 1.0, 1.0}, {2.0, 2.5, 6.0, 0.3}, {1.0, -2.5, 0.3, 3.0},
                               {-2.5, -2.5, 1.5, 1.5}, {-3.0, 1.0, 0.3, 3.0}};
    for (const auto& b : boxes)
    {
        map.setOccupiedRect(b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2);
    }
    map.inflate(0.57);
    Scenario s{"nav_world " + std::to_string(cells) + "x" + std::to_string(cells), map, 0, 0, 0, 0};
    map.worldToGrid(0.0, 0.0, s.start_row, s.start_col);
    map.worldToGrid(4.0, 4.0, s.goal_row, s.goal_col);
    return s;
}

// 随机障碍物地图（固定随机种子，每次运行结果一样），从左下角到右上角
Scenario makeRandom(int cells, double density, unsigned seed)
{
    GridMap map(cells, cells, 0.1, 0.0, 0.0);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    for (int r = 0; r < cells; ++r)
    {
        for (int c = 0; c < cells; ++c)
        {
            bool near_corner = (r + c < 4) || (r + c > 2 * cells - 6);
            if (!near_corner && uniform(rng) < density)
            {
                map.setOccupied(r, c);
            }
        }
    }
    char name[64];
    std::snprintf(name, sizeof(name), "random %dx%d (%.0f%%)", cells, cells, density * 100);
    return Scenario{name, map, 0, 0, cells - 1, cells - 1};
}

// 运行 repeat 次，返回中位数耗时（毫秒）
double medianMs(const std::function<Path()>& run, int repeat, Path& result)
{
    std::vector<double> times;
    for (int i = 0; i < repeat; ++i)
    {
        auto t0 = std::chrono::steady_clock::now();
        result = run();
        times.push_back(std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count());
    }
    std::sort(times.begin(), times.end());
    return times[times.size() / 2];
}

int main()
{
    std::vector<Scenario> scenarios = {
        makeNavWorld(0.5), makeNavWorld(0.1), makeNavWorld(0.05), makeRandom(200, 0.25, 42),
    };

    std::printf("%-22s | %12s | %12s %8s | %12s | %s\n", "scenario", "basic 4-dir", "new 4-dir",
                "speedup", "new 8-dir", "path length 4-dir -> 8-dir");
    std::printf("%s\n", std::string(104, '-').c_str());

    for (auto& s : scenarios)
    {
        Path basic, new4, new8;
        double t_basic = medianMs([&] {
            return astarSearchBasic(s.map, s.start_row, s.start_col, s.goal_row, s.goal_col); }, 3, basic);
        double t_new4 = medianMs([&] {
            return astarSearch(s.map, s.start_row, s.start_col, s.goal_row, s.goal_col, false); }, 21, new4);
        double t_new8 = medianMs([&] {
            return astarSearch(s.map, s.start_row, s.start_col, s.goal_row, s.goal_col, true); }, 21, new8);

        double res = s.map.resolution();
        std::printf("%-22s | %9.3f ms | %9.3f ms %7.0fx | %9.3f ms | %.2f m -> %.2f m (%.0f%% shorter)%s\n",
                    s.name.c_str(), t_basic, t_new4, t_basic / t_new4, t_new8,
                    pathLength(new4) * res, pathLength(new8) * res,
                    100.0 * (1.0 - pathLength(new8) / pathLength(new4)),
                    (pathLength(basic) == pathLength(new4)) ? "" : "  [!] basic/new length differ");
    }
    return 0;
}
