#pragma once

// 控制器参数
struct ControllerParams
{
    double k_linear = 0.5;          // 线速度比例系数：离目标越远，开得越快
    double k_angular = 1.5;         // 角速度比例系数：角度误差越大，转得越快
    double max_linear = 0.3;        // 线速度上限 (m/s)
    double max_angular = 1.0;       // 角速度上限 (rad/s)
    double turn_in_place_angle = 0.5;  // 角度误差超过它 (约 29°) 时，先原地转向，不前进
};

// 速度命令：对应 Twist 里的 linear.x 和 angular.z
struct VelocityCommand
{
    double linear = 0.0;
    double angular = 0.0;
};

// P 控制器（比例控制）：根据距离和角度误差计算速度命令
VelocityCommand computeVelocity(double distance, double angle_error,
                                const ControllerParams& params);
