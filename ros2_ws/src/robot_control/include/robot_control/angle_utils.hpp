#pragma once

// 把四元数 (x, y, z, w) 转换成 yaw 角（弧度，范围 -π ~ π）
double quaternionToYaw(double x, double y, double z, double w);

// 把任意角度（弧度）归一化到 -π ~ π
double normalizeAngle(double angle);