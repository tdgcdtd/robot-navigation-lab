#include "robot_control/controller.hpp"
#include <algorithm>
#include <cmath>

VelocityCommand computeVelocity(double distance, double angle_error,
                                const ControllerParams& params)
{
    VelocityCommand cmd;

    // 1. 角速度：与角度误差成正比，并限制在 ±max_angular 之内
    cmd.angular = std::clamp(params.k_angular * angle_error,
                             -params.max_angular, params.max_angular);

    // 2. 线速度：车头还没大致对准目标时，先原地转，不前进
    if (std::abs(angle_error) > params.turn_in_place_angle)
    {
        cmd.linear = 0.0;
    }
    else
    {
        // 与距离成正比，越接近目标越慢，并限制在 max_linear 之内
        cmd.linear = std::min(params.k_linear * distance, params.max_linear);
    }

    return cmd;
}
