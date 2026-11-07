// P2 核心手写：E6 机械臂正运动学（FK）—— 6 个关节角 → 末端（Link6）在底座坐标系里的位置
// 编译运行（WSL 里就行，不用进容器，也不需要 ROS）：
//   cd ~/project/robot-navigation-lab/practice
//   g++ -std=c++17 p2_fk.cpp -o p2_fk && ./p2_fk
//
// 要写的 5 个函数（按顺序写，每写完一个就可以编译运行，看对应的测试是否通过）：
//   ① multiply      4×4 矩阵乘法
//   ② rotX / rotY / rotZ   绕 x / y / z 轴旋转 angle 的 4×4 矩阵（平移部分为 0）
//   ③ fromXyzRpy    URDF 的 origin（xyz + rpy）→ 4×4 齐次矩阵
//   ④ forwardKinematics   6 个关节连乘，得到末端的 4×4 矩阵
#include <array>
#include <cmath>
#include <cstdio>

using Mat4 = std::array<std::array<double, 4>, 4>;  // m[行][列]

Mat4 identity()
{
    Mat4 m{};  // 全部初始化为 0
    for (int i = 0; i < 4; ++i)
    {
        m[i][i] = 1.0;
    }
    return m;
}

// 一个关节在 URDF 里的 origin：child 坐标系相对 parent 坐标系的平移 xyz 和旋转 rpy
// E6 的 6 个关节都绕各自坐标系的 z 轴转（URDF 里 axis 都是 0 0 1）
struct JointOrigin
{
    double x, y, z;           // 平移（米）
    double roll, pitch, yaw;  // 旋转（弧度），分别绕 x、y、z 轴
};

// 直接抄自 me6_robot.urdf（dummy_joint 是固定关节、origin 为 0，所以省略）
const std::array<JointOrigin, 6> kJoints = {{
    {0.0,    0.0,     0.1268, 0.0,     0.0,    0.0},      // joint1: base_link → Link1
    {-0.046, 0.0,     0.04,   1.5708,  0.0,    -1.5708},  // joint2: Link1 → Link2
    {0.0,    0.18906, 0.003,  0.0,     0.0,    0.0},      // joint3: Link2 → Link3
    {0.0,    0.16,    0.005,  0.0,     0.0,    0.0},      // joint4: Link3 → Link4
    {0.0,    0.067,   0.032,  -1.5708, 1.5708, 0.0},      // joint5: Link4 → Link5
    {-0.047, 0.0,     0.034,  1.5708,  0.0,    -1.5708},  // joint6: Link5 → Link6
}};

// ① 矩阵乘法：返回 a · b
Mat4 multiply(const Mat4& a, const Mat4& b)
{
    Mat4 c{};
    // ========== 在这里写你的代码 ==========
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            for(int k = 0; k < 4; k++)
            {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    // =====================================
    return c;
}

// ② 绕 x / y / z 轴旋转 angle（弧度）的齐次矩阵：左上 3×3 是旋转，平移为 0
Mat4 rotX(double angle)
{
    Mat4 m = identity();
    // ========== 在这里写你的代码 ==========
    m = {{
        {1, 0, 0, 0},
        {0, std::cos(angle), std::sin(-angle), 0},
        {0, std::sin(angle), std::cos(angle), 0},
        {0, 0, 0, 1},
    }};
    // =====================================
    return m;
}

Mat4 rotY(double angle)
{
    Mat4 m = identity();
    // ========== 在这里写你的代码 ==========
    m = {{
        {std::cos(angle), 0, std::sin(angle), 0},
        {0, 1, 0, 0},
        {std::sin(-angle), 0, std::cos(angle), 0},
        {0, 0, 0, 1},
    }};
    // =====================================
    return m;
}

Mat4 rotZ(double angle)
{
    Mat4 m = identity();
    // ========== 在这里写你的代码 ==========
    m = {{
        {std::cos(angle), std::sin(-angle), 0, 0},
        {std::sin(angle), std::cos(angle), 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
    }};
    // =====================================
    return m;
}

// ③ URDF origin → 齐次矩阵
// 提示：URDF 规定 rpy 的旋转矩阵 R = Rz(yaw) · Ry(pitch) · Rx(roll)；平移放在最后一列
Mat4 fromXyzRpy(const JointOrigin& o)
{
    Mat4 m = identity();
    // ========== 在这里写你的代码 ==========
    m = multiply(multiply(rotZ(o.yaw), rotY(o.pitch)), rotX(o.roll));
    m[0][3] = o.x;
    m[1][3] = o.y;
    m[2][3] = o.z;
    // =====================================
    return m;
}

// ④ 正运动学：q[i] 是第 i+1 个关节的角度（弧度），返回 Link6 相对 base_link 的齐次矩阵
// 提示：每个关节的变换 = "固定的 origin" 再 "绕 z 轴转关节角"
Mat4 forwardKinematics(const std::array<double, 6>& q)
{
    Mat4 t = identity();
    // ========== 在这里写你的代码 ========== 
    for(int i = 0; i < 6; i++)
    {
        Mat4 p = identity();
        p = multiply(fromXyzRpy(kJoints[i]), rotZ(q[i]));
        t = multiply(t, p);
    }
    // =====================================
    return t;
}

// ---------------- 下面是测试，不用改 ----------------
int failures = 0;

void expectNear(const char* name, double got, double want, double tol)
{
    const bool ok = std::fabs(got - want) <= tol;
    if (!ok)
    {
        ++failures;
    }
    std::printf("  %s %-40s 得到 %9.5f  期望 %9.5f\n", ok ? "✅" : "❌", name, got, want);
}

void expectMat(const char* name, const Mat4& got, const Mat4& want)
{
    bool ok = true;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            ok = ok && std::fabs(got[i][j] - want[i][j]) < 1e-3;
        }
    }
    if (!ok)
    {
        ++failures;
    }
    std::printf("  %s %s\n", ok ? "✅" : "❌", name);
    if (!ok)
    {
        for (int i = 0; i < 4; ++i)
        {
            std::printf("      得到 [%7.3f %7.3f %7.3f %7.3f]   期望 [%7.3f %7.3f %7.3f %7.3f]\n",
                        got[i][0], got[i][1], got[i][2], got[i][3],
                        want[i][0], want[i][1], want[i][2], want[i][3]);
        }
    }
}

void checkPosition(const char* name, const std::array<double, 6>& q, double x, double y, double z)
{
    const Mat4 t = forwardKinematics(q);
    const double err = std::sqrt(std::pow(t[0][3] - x, 2) + std::pow(t[1][3] - y, 2) + std::pow(t[2][3] - z, 2));
    const bool ok = err < 0.001;  // 验收标准：误差 < 1 mm
    if (!ok)
    {
        ++failures;
    }
    std::printf("  %s %-22s 得到 (%7.4f, %7.4f, %7.4f)  期望 (%7.4f, %7.4f, %7.4f)  误差 %.2f mm\n",
                ok ? "✅" : "❌", name, t[0][3], t[1][3], t[2][3], x, y, z, err * 1000);
}

int main()
{
    const double kPi = 3.14159265358979;

    std::printf("① multiply\n");
    Mat4 a = identity();
    a[0][3] = 1.0;  // 沿 x 平移 1
    Mat4 b = identity();
    b[1][3] = 2.0;  // 沿 y 平移 2
    Mat4 ab = identity();
    ab[0][3] = 1.0;
    ab[1][3] = 2.0;
    expectMat("单位矩阵 × A = A", multiply(identity(), a), a);
    expectMat("两次平移相乘 = 平移相加", multiply(a, b), ab);
    Mat4 r90 = Mat4{{{0, -1, 0, 0}, {1, 0, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    Mat4 r90_then_x = Mat4{{{0, -1, 0, 0}, {1, 0, 0, 1}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    expectMat("先转 90° 再沿新 x 平移 1（顺序很重要）", multiply(r90, a), r90_then_x);

    std::printf("② rotX / rotY / rotZ（转 90°）\n");
    expectMat("rotZ(90°)", rotZ(kPi / 2), r90);
    expectMat("rotX(90°)", rotX(kPi / 2), Mat4{{{1, 0, 0, 0}, {0, 0, -1, 0}, {0, 1, 0, 0}, {0, 0, 0, 1}}});
    expectMat("rotY(90°)", rotY(kPi / 2), Mat4{{{0, 0, 1, 0}, {0, 1, 0, 0}, {-1, 0, 0, 0}, {0, 0, 0, 1}}});

    std::printf("③ fromXyzRpy（用 URDF 里的真实数据）\n");
    expectMat("joint1 origin（只平移）", fromXyzRpy(kJoints[0]),
              Mat4{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0.1268}, {0, 0, 0, 1}}});
    expectMat("joint2 origin（roll + yaw）", fromXyzRpy(kJoints[1]),
              Mat4{{{0, 0, -1, -0.046}, {-1, 0, 0, 0}, {0, 1, 0, 0.04}, {0, 0, 0, 1}}});
    expectMat("joint5 origin（roll + pitch）", fromXyzRpy(kJoints[4]),
              Mat4{{{0, -1, 0, 0}, {0, 0, 1, 0.067}, {-1, 0, 0, 0.032}, {0, 0, 0, 1}}});

    std::printf("④ forwardKinematics（期望值来自 tf2_echo 和 MoveIt 的实测，误差要 < 1 mm）\n");
    checkPosition("全部归零", {0, 0, 0, 0, 0, 0}, -0.1330, 0.0000, 0.6169);
    checkPosition("home 姿态", {0, 0.5337, 0.7276, 0.3548, -1.5848, 0}, -0.0853, 0.3473, 0.3268);
    checkPosition("任意姿态 A", {0.5, -0.3, 0.8, 0.2, 1.0, -0.7}, -0.1244, -0.0046, 0.5906);
    checkPosition("任意姿态 B", {-1.2, 0.9, -1.1, 0.6, -0.5, 2.0}, 0.1183, 0.1825, 0.5254);
    checkPosition("任意姿态 C", {2.0, -1.0, 1.5, -0.8, 0.3, 0.4}, 0.1686, -0.0668, 0.5017);
    expectMat("归零时末端的完整矩阵（含朝向）", forwardKinematics({0, 0, 0, 0, 0, 0}),
              Mat4{{{0, 0, -1, -0.1330}, {-1, 0, 0, 0}, {0, 1, 0, 0.6169}, {0, 0, 0, 1}}});

    std::printf(failures == 0 ? "\n全部通过 🎉\n" : "\n有 %d 项没通过\n", failures);
    return failures == 0 ? 0 : 1;
}
