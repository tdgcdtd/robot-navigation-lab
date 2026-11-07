#include "path_planner/grid_map.hpp"
#include "path_planner/astar_planner.hpp"
#include <iostream>

int main()
{
    GridMap grid_map(20, 20, 0.5, -5.0, -5.0);

    // grid_map.setOccupied(11, 13);
    // grid_map.setOccupied(11, 14);
    // grid_map.setOccupied(12, 13);
    // grid_map.setOccupied(12, 14);
    grid_map.setOccupiedRect(1.5, 0.5, 2.5, 1.5);
    auto path = astarSearch(grid_map, 11, 12, 11, 15);

    grid_map.print();
    for (const auto &point : path)
    {
        double x;
        double y;

        // 调用 gridToWorld()，得到 x 和 y
        grid_map.gridToWorld(point.first, point.second, x, y);
        // 输出世界坐标
        std::cout << "World coordinates: (" << x << ", " << y << ")" << std::endl;
    }

    std::cout << std::endl;
    return 0;
}
