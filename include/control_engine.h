#ifndef CONTROL_ENGINE_H
#define CONTROL_ENGINE_H

#include "types.h"              // 数据结构
#include "robot_interface.h"    // 抽象接口（注意：没有包含任何 JAKA 头文件！）
#include "data_recorder.h"      // 数据记录器
#include <random>               // C++ 随机数库

// ========================================
// 【控制引擎】：核心算法逻辑
// 
// 这个类实现了"生成随机点位 → 运动到位 → 等待静止 → 采集数据"的完整流程。
// 它只认识 IRobotInterface（抽象接口），不认识 JAKA。
// 这意味着：如果以后换别的品牌机器人，只要写一个新的接口实现类，
// 控制引擎代码完全不用改！
// ========================================

class ControlEngine {
public:
    // 构造函数需要传入两个指针：
    //   robot：机器人接口（用于运动控制和数据采集）
    //   recorder：数据记录器（用于写入 CSV）
    ControlEngine(IRobotInterface* robot, DataRecorder* recorder);

    // ================== 参数设置函数 ==================
    // 这些函数用来配置实验参数，必须在 run() 之前调用
    
    // 设置关节运动范围（最小角度、最大角度）
    void setJointLimits(const JointAngles& minAngles, const JointAngles& maxAngles);
    
    // 设置要采集的随机点位数量（比如 50 个点）
    void setPointCount(int count);
    
    // 设置到位后的等待时间（秒），让机器人完全静止
    void setSettlingTime(double seconds);
    
    // 设置"静止判定"的速度阈值（rad/s）
    void setVelocityThreshold(double threshold);
    
    // 设置运动速度百分比（%）
    void setMaxVelocity(double maxVel);
    
    // 设置 Home 位（起始/结束位置）
    void setHomePosition(const JointAngles& home);

    // ================== 主控函数 ==================
    
    // 执行完整的随机点位采集流程
    // 返回值：true=成功完成，false=执行过程中出错
    bool run();

private:
    // 【成员变量】
    IRobotInterface* robot_;        // 机器人接口指针（多态：实际指向 JakaRobotInterface）
    DataRecorder* recorder_;        // 数据记录器指针
    
    JointAngles minAngles_, maxAngles_, homePosition_;  // 关节限位和 Home 位
    int pointCount_;                // 随机点数量
    double settlingTime_;           // 静止等待时间
    double velocityThreshold_;      // 静止速度阈值
    double maxVelocity_;            // 运动速度

    // 随机数生成器相关
    std::mt19937 rng_;              // Mersenne Twister 随机数引擎（高质量随机数）
    std::uniform_real_distribution<double> dist_;  // 均匀分布：生成 0~1 之间的随机小数

    // 【私有辅助函数】
    
    // 生成一个范围内的随机关节角度
    JointAngles generateRandomPoint();
    
    // 等待机器人静止（先睡一段时间，再检查速度）
    bool waitForSettling();
    
    // 在当前位置采集一组数据并记录到 CSV
    bool collectAtPoint(int pointIndex);
    
    // 回到 Home 位
    bool returnToHome();
};

#endif
