#ifndef TYPES_H
#define TYPES_H

#include <array>

struct JointAngles {
    std::array<double, 6> j;
    JointAngles() : j{0,0,0,0,0,0} {}
    JointAngles(double j1, double j2, double j3, double j4, double j5, double j6)
        : j{j1, j2, j3, j4, j5, j6} {}
};

struct JointTorques {
    std::array<double, 6> t;
    JointTorques() : t{0,0,0,0,0,0} {}
};

struct SixAxisForce {
    double fx, fy, fz;  // 力 (N)
    double tx, ty, tz;  // 力矩 (Nm)
    SixAxisForce() : fx(0), fy(0), fz(0), tx(0), ty(0), tz(0) {}
};

struct RobotDataSample {
    double timestamp;       // 秒
    JointAngles angles;
    JointTorques torques;
    SixAxisForce force;
    int pointIndex;         // 点位编号
};

#endif
