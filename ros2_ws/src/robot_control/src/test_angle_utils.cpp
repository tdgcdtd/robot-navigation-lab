#include "robot_control/angle_utils.hpp"
#include <iostream>
#include <cmath>

int main()
{
    std::cout << "=== quaternionToYaw ===" << std::endl;
    std::cout << "facing +x: " << quaternionToYaw(0, 0, 0, 1) << std::endl;
    // TODO: 补上表格里另外三个朝向（+y、-x、-y）
    std::cout << "facing +y: " << quaternionToYaw(0, 0, std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0) << '\n';
    std::cout << "facing -x: " << quaternionToYaw(0, 0, 1, 0) << '\n';
    std::cout << "facing -y: " << quaternionToYaw(0, 0, -std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0) << '\n';

    std::cout << "=== normalizeAngle ===" << std::endl;
    //   3π/2      → 应该得到 -π/2  (≈ -1.5708)
    std::cout << "3π/2: " << normalizeAngle(3*M_PI/2) << std::endl;
    //  -3π/2      → 应该得到  π/2  (≈  1.5708)
    std::cout << "-3π/2:" << normalizeAngle(-3*M_PI/2) << '\n';
    //   2π + 0.5  → 应该得到  0.5
    std::cout << "2π + 0.5:" << normalizeAngle(2*M_PI + 0.5) << '\n';
    //   0.3       → 应该得到  0.3
    std::cout << "0.3: " << normalizeAngle(0.3) << '\n';

    // TODO: 用知识点 4 的场景测试：
    //   yaw = 170°，目标 = -170°，算出误差并归一化，应该得到 +20°
    //   提示：度数转弧度 = 度数 * M_PI / 180.0
    double error = normalizeAngle(-170.0*M_PI / 180.0 - 170.0*M_PI / 180.0);
    std::cout << "error(rad) = " << error << '\n'
              << "error(deg) = " << error * 180.0 / M_PI << '\n';

    return 0;
}