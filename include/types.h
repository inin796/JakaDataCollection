// TYPES_H 是一个"宏保护"（Macro Guard）
// 它的作用是防止同一个头文件被重复包含，避免编译错误
#ifndef TYPES_H
#define TYPES_H

// 引入标准库中的数组容器 array
// array 是 C++ 提供的一种固定大小的数组，比传统 C 语言数组更安全
#include <array>

// ========================================
// 结构体（struct）：用来把多个相关的数据打包在一起
// 就像 Excel 里的一行数据，有多个列
// ========================================

// 【关节角度】结构体
// 机械臂有 6 个关节，每个关节有一个角度（单位：弧度 rad）
struct JointAngles {
    // std::array<double, 6> 表示：存放 6 个 double（双精度浮点数）的固定数组
    // j[0] ~ j[5] 分别对应关节 1~6 的角度
    std::array<double, 6> j;
    
    // 【构造函数】：创建对象时自动执行的初始化函数
    // 这里把所有关节角度初始化为 0，防止出现随机垃圾值
    JointAngles() : j{0,0,0,0,0,0} {}
    
    // 【带参数的构造函数】：允许创建时直接指定 6 个角度
    // 例如：JointAngles pos(0.5, 1.2, -0.3, 0, 0, 0);
    JointAngles(double j1, double j2, double j3, double j4, double j5, double j6)
        : j{j1, j2, j3, j4, j5, j6} {}
};

// 【关节扭矩】结构体
// 扭矩可以理解为电机输出的"力气"，单位是牛顿米（Nm）
struct JointTorques {
    std::array<double, 6> t;  // t[0]~t[5] 对应 6 个关节的扭矩
    JointTorques() : t{0,0,0,0,0,0} {}
};

// 【六维力/力矩】结构体
// 机械臂末端（比如夹爪处）安装的力传感器可以测量：
//   fx, fy, fz：三个方向的力（Force，单位 N）
//   tx, ty, tz：三个方向的力矩（Torque，单位 Nm）
struct SixAxisForce {
    double fx, fy, fz;
    double tx, ty, tz;
    SixAxisForce() : fx(0), fy(0), fz(0), tx(0), ty(0), tz(0) {}
};

// 【数据采样点】结构体
// 这是我们要记录到 CSV 文件里的"一行完整数据"
struct RobotDataSample {
    double timestamp;       // 时间戳：记录这是第几秒采集的数据
    JointAngles angles;   // 6 个关节角度
    JointTorques torques; // 6 个关节扭矩
    SixAxisForce force;   // 末端六维力/力矩
    int pointIndex;       // 点位编号：第几个随机点（方便后期分析）
};

#endif // TYPES_H
