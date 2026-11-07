#include "robot_control/angle_utils.hpp"
#include <cmath>

double quaternionToYaw(double x, double y, double z, double w)
{
    // TODO: 按照知识点 3 的公式计算并返回 yaw
    double yaw  = std::atan2( 2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
    return yaw;
}

double normalizeAngle(double angle)
{
    // TODO: 按照知识点 4 的规则，用 while 循环把 angle 调整到 -π ~ π
    // 提示：π 在 C++ 里写作 M_PI
    while (angle > M_PI)
    {
        angle -= 2*M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2*M_PI;
    }

    return angle;
}