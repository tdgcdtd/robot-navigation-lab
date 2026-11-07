// P1 核心手写 ①：P 控制器 computeVelocity
// 编译运行：
//   cd ~/project/robot-navigation-lab/practice
//   g++ -std=c++17 p1_compute_velocity.cpp -o p1_compute_velocity && ./p1_compute_velocity
#include <algorithm>
#include <cmath>
#include <cstdio>

struct ControllerParams
{
    double k_linear = 0.5;             // 线速度比例系数
    double k_angular = 1.5;            // 角速度比例系数
    double max_linear = 0.3;           // 线速度上限 (m/s)
    double max_angular = 1.0;          // 角速度上限 (rad/s)
    double turn_in_place_angle = 0.5;  // 角度误差超过它 (rad) 时，先原地转向、不前进
};

struct VelocityCommand
{
    double linear = 0.0;   // 前进速度 (m/s)，对应 Twist 的 linear.x
    double angular = 0.0;  // 转向速度 (rad/s)，对应 Twist 的 angular.z；正数 = 逆时针（左转）
};

// 输入：distance     到目标点的距离 (m)，总是 >= 0
//       angle_error  车头还要转多少才能对准目标 (rad)，已归一化到 [-π, π]；正数 = 目标在左边
//       params       控制器参数
// 输出：这一次应该发出的速度命令
VelocityCommand computeVelocity(double distance, double angle_error,
                                const ControllerParams& params)
{
    VelocityCommand cmd;

    // ========== 在这里写你的代码 ==========
    cmd.angular = std::clamp(params.k_angular * angle_error, -params.max_angular, params.max_angular);
    // if(angle_error > 0)
    // {
    //     cmd.angular = std::min(params.k_angular * angle_error, params.max_angular);
    // }
    // else
    // {
    //     cmd.angular = std::max(params.k_angular * angle_error, -params.max_angular);
    // }
    
    if(std::abs(angle_error) > params.turn_in_place_angle)
    {
        cmd.linear = 0.0;
    }
    else
    {
        cmd.linear = std::min(params.k_linear * distance, params.max_linear);
    }

    // =====================================

    return cmd;
}

// ---------------- 下面是测试，不用改 ----------------
int failures = 0;

void check(const char* name, double distance, double angle_error,
           double expect_linear, double expect_angular)
{
    ControllerParams params;
    VelocityCommand cmd = computeVelocity(distance, angle_error, params);
    bool ok = std::abs(cmd.linear - expect_linear) < 1e-6 &&
              std::abs(cmd.angular - expect_angular) < 1e-6;
    if (!ok) ++failures;
    std::printf("%s %-28s 距离=%.2f 误差=%+.2f -> linear=%.3f angular=%+.3f   (期望 linear=%.3f angular=%+.3f)\n",
                ok ? "✅" : "❌", name, distance, angle_error,
                cmd.linear, cmd.angular, expect_linear, expect_angular);
}

int main()
{
    check("1 普通情况", 0.4, 0.17, 0.2, 0.255);
    check("2 车头偏太多：原地转+限幅", 2.0, 0.70, 0.0, 1.0);
    check("3 已对准、很远：线速度限幅", 2.0, 0.0, 0.3, 0.0);
    check("4 目标在右边：负方向限幅", 1.0, -0.70, 0.0, -1.0);
    check("5 很近：慢慢开", 0.1, 0.4, 0.05, 0.6);
    check("6 误差正好等于阈值", 1.0, 0.5, 0.3, 0.75);
    std::printf(failures == 0 ? "\n🎉 全部通过！\n" : "\n还有 %d 个没通过，继续加油\n", failures);
    return failures == 0 ? 0 : 1;
}
