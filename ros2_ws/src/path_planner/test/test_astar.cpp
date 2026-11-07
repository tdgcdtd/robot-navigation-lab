// A* 与栅格地图的单元测试（GoogleTest）
// 运行：colcon test --packages-select path_planner --event-handlers console_direct+
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "path_planner/astar_basic.hpp"
#include "path_planner/astar_planner.hpp"
#include "path_planner/grid_map.hpp"

using Path = std::vector<std::pair<int, int>>;

// 10 x 10 的空地图，每格 1 m，原点 (0, 0)：格子坐标和米数一致，方便心算
GridMap emptyMap(int size = 10)
{
    return GridMap(size, size, 1.0, 0.0, 0.0);
}

// 检查路径：首尾正确、每一步只移动到相邻格子、不经过不可通行的格子
void expectValidPath(const GridMap& map, const Path& path, std::pair<int, int> start,
                     std::pair<int, int> goal)
{
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), goal);
    for (size_t i = 1; i < path.size(); ++i)
    {
        EXPECT_LE(std::abs(path[i].first - path[i - 1].first), 1);
        EXPECT_LE(std::abs(path[i].second - path[i - 1].second), 1);
        EXPECT_FALSE(map.isOccupied(path[i].first, path[i].second))
            << "path goes through (" << path[i].first << ", " << path[i].second << ")";
    }
}

TEST(Heuristic, ManhattanAndOctile)
{
    EXPECT_DOUBLE_EQ(manhattanHeuristic(0, 0, 3, 4), 7.0);
    EXPECT_DOUBLE_EQ(octileHeuristic(0, 0, 0, 5), 5.0);                // 纯直走
    EXPECT_DOUBLE_EQ(octileHeuristic(0, 0, 3, 3), 3.0 * std::sqrt(2.0)); // 纯斜走
    EXPECT_DOUBLE_EQ(octileHeuristic(0, 0, 3, 5), 2.0 + 3.0 * std::sqrt(2.0));
}

TEST(AStar, EmptyMapFourDirectionsIsManhattanLength)
{
    GridMap map = emptyMap();
    Path path = astarSearch(map, 0, 0, 6, 8, false);
    expectValidPath(map, path, {0, 0}, {6, 8});
    EXPECT_DOUBLE_EQ(pathLength(path), 14.0);
}

TEST(AStar, EmptyMapEightDirectionsIsOctileLength)
{
    GridMap map = emptyMap();
    Path path = astarSearch(map, 0, 0, 6, 8, true);
    expectValidPath(map, path, {0, 0}, {6, 8});
    EXPECT_NEAR(pathLength(path), octileHeuristic(0, 0, 6, 8), 1e-9);  // 空地图上应达到下界
}

TEST(AStar, StartEqualsGoal)
{
    GridMap map = emptyMap();
    Path path = astarSearch(map, 3, 3, 3, 3);
    ASSERT_EQ(path.size(), 1u);
    EXPECT_EQ(path.front(), std::make_pair(3, 3));
}

TEST(AStar, GoesAroundWall)
{
    // 第 5 列是一堵墙，只在第 9 行留了缺口
    GridMap map = emptyMap();
    for (int row = 0; row < 9; ++row)
    {
        map.setOccupied(row, 5);
    }
    for (bool diagonal : {false, true})
    {
        Path path = astarSearch(map, 0, 0, 0, 9, diagonal);
        expectValidPath(map, path, {0, 0}, {0, 9});
    }
}

TEST(AStar, NoPathWhenGoalEnclosed)
{
    GridMap map = emptyMap();
    for (int i = 4; i <= 6; ++i)  // 用一圈障碍物把 (5, 5) 围起来
    {
        map.setOccupied(4, i);
        map.setOccupied(6, i);
        map.setOccupied(i, 4);
        map.setOccupied(i, 6);
    }
    EXPECT_TRUE(astarSearch(map, 0, 0, 5, 5, true).empty());
    EXPECT_TRUE(astarSearch(map, 0, 0, 5, 5, false).empty());
}

TEST(AStar, NoPathWhenGoalOccupiedOrOutside)
{
    GridMap map = emptyMap();
    map.setOccupied(7, 7);
    EXPECT_TRUE(astarSearch(map, 0, 0, 7, 7).empty());
    EXPECT_TRUE(astarSearch(map, 0, 0, 20, 20).empty());
}

TEST(AStar, NoCornerCutting)
{
    // 两个障碍物斜着相邻：(1,0) 和 (0,1)。
    // 从 (0,0) 斜着走到 (1,1) 会贴着两个障碍物的拐角挤过去，必须禁止。
    GridMap map(3, 3, 1.0, 0.0, 0.0);
    map.setOccupied(1, 0);
    map.setOccupied(0, 1);
    EXPECT_TRUE(astarSearch(map, 0, 0, 1, 1, true).empty());
}

TEST(AStar, SameLengthAsBasicVersionOnRandomMaps)
{
    // 两个版本都是最优算法，在 4 邻域下找到的路径长度必须相同
    for (unsigned seed = 1; seed <= 20; ++seed)
    {
        GridMap map(30, 30, 1.0, 0.0, 0.0);
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        for (int r = 0; r < 30; ++r)
        {
            for (int c = 0; c < 30; ++c)
            {
                if (r + c > 0 && r + c < 58 && uniform(rng) < 0.3)
                {
                    map.setOccupied(r, c);
                }
            }
        }
        Path basic = astarSearchBasic(map, 0, 0, 29, 29);
        Path fast = astarSearch(map, 0, 0, 29, 29, false);
        EXPECT_EQ(basic.empty(), fast.empty()) << "seed " << seed;
        EXPECT_DOUBLE_EQ(pathLength(basic), pathLength(fast)) << "seed " << seed;
    }
}

TEST(PathPostProcess, SimplifyKeepsOnlyCorners)
{
    Path path = {{0, 0}, {0, 1}, {0, 2}, {1, 2}, {2, 2}};
    Path expected = {{0, 0}, {0, 2}, {2, 2}};
    EXPECT_EQ(simplifyPath(path), expected);
}

TEST(PathPostProcess, SmoothPathKeepsEndpointsAndLineOfSight)
{
    GridMap map = emptyMap();
    for (int row = 0; row < 7; ++row)
    {
        map.setOccupied(row, 5);
    }
    Path raw = astarSearch(map, 0, 0, 0, 9, true);
    Path smooth = smoothPath(map, raw);
    ASSERT_GE(smooth.size(), 2u);
    EXPECT_LT(smooth.size(), raw.size());
    EXPECT_EQ(smooth.front(), raw.front());
    EXPECT_EQ(smooth.back(), raw.back());
    for (size_t i = 1; i < smooth.size(); ++i)
    {
        EXPECT_TRUE(hasLineOfSight(map, smooth[i - 1], smooth[i]));
    }
    // 在空地图上，起点能直接看到终点：拉直后只剩 2 个点
    GridMap empty = emptyMap();
    EXPECT_EQ(smoothPath(empty, astarSearch(empty, 0, 0, 6, 8, true)).size(), 2u);
}

TEST(PathPostProcess, LineOfSightCatchesCornerClip)
{
    // 线段 (0,0) -> (7,11) 只擦过格子 (2,2) 的一角（约 0.17 格长）。
    // 每隔 0.2 格采样的旧实现会漏掉它；精确遍历必须发现它。
    GridMap map(20, 20, 1.0, 0.0, 0.0);
    map.setOccupied(2, 2);
    EXPECT_FALSE(hasLineOfSight(map, {0, 0}, {7, 11}));
    EXPECT_FALSE(hasLineOfSight(map, {7, 11}, {0, 0}));  // 反方向也一样
    EXPECT_TRUE(hasLineOfSight(map, {0, 0}, {0, 11}));   // 不经过 (2,2) 的线是通的
}

TEST(PathPostProcess, LineOfSightThroughCornerNeedsBothSidesFree)
{
    // 线段 (0,0) -> (2,2) 正好穿过格子角点 (1,1)。旁边的 (1,0) 有障碍物 -> 不能通过
    GridMap map(5, 5, 1.0, 0.0, 0.0);
    EXPECT_TRUE(hasLineOfSight(map, {0, 0}, {2, 2}));
    map.setOccupied(1, 0);
    EXPECT_FALSE(hasLineOfSight(map, {0, 0}, {2, 2}));
}

TEST(GridMap, InflateMarksCellsWithinRadius)
{
    GridMap map = emptyMap();
    map.setOccupied(5, 5);
    map.inflate(1.0);  // 1 m = 1 格
    EXPECT_EQ(map.getCost(5, 5), GridMap::kOccupied);
    EXPECT_EQ(map.getCost(5, 6), GridMap::kInflated);  // 正旁边：距离 0.5
    EXPECT_EQ(map.getCost(6, 6), GridMap::kInflated);  // 斜旁边：距离约 0.71
    EXPECT_EQ(map.getCost(5, 7), GridMap::kFree);      // 隔一格：距离 1.5
}
