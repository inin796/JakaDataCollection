#include "control_engine.h"
#include <iostream>     // 打印信息
#include <thread>       // 休眠
#include <chrono>       // 时间单位
#include <cmath>        // M_PI（π 常量）

// ========================================
// 【构造函数】
// 初始化所有成员变量。
// 
// 注意：robot 和 recorder 是"指针"（存储的是内存地址，不是对象本身）。
// 控制引擎不拥有这两个对象，只是"借用"它们。
// ========================================
ControlEngine::ControlEngine(IRobotInterface* robot, DataRecorder* recorder)
    : robot_(robot), recorder_(recorder),       // 保存指针
      pointCount_(50),                          // 默认 50 个点
      settlingTime_(2.0),                       // 默认等待 2 秒
      velocityThreshold_(0.001),                // 默认速度阈值 0.001 rad/s
      maxVelocity_(20.0),                        // 默认速度 20%
      rng_(std::random_device{}()),             // 用硬件随机数初始化随机引擎
      dist_(0.0, 1.0)                            // 均匀分布：生成 0.0~1.0 的随机数
{
    // 设置默认关节限位（单位：弧度）
    // 这些值是 JAKA 某型号机械臂的典型限位，请根据你的实际机型修改！
    // -2.967 rad ≈ -170°
    minAngles_ = JointAngles(-2.967, -1.745, -2.967, -3.316, -2.967, -3.316);
    maxAngles_ = JointAngles( 2.967,  1.745,  2.967,  3.316,  2.967,  3.316);
    
    // 默认 Home 位
    // M_PI/2 = π/2 ≈ 1.5708 rad ≈ 90°
    homePosition_ = JointAngles(0, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0);
}

// ========================================
// 【参数设置函数】
// 这些函数只是简单地给成员变量赋值。
// "const JointAngles&" 表示"常量引用"：只读、不拷贝。
// ========================================
void ControlEngine::setJointLimits(const JointAngles& minAngles, const JointAngles& maxAngles) {
    minAngles_ = minAngles;
    maxAngles_ = maxAngles;
}

void ControlEngine::setPointCount(int count) { pointCount_ = count; }
void ControlEngine::setSettlingTime(double seconds) { settlingTime_ = seconds; }
void ControlEngine::setVelocityThreshold(double threshold) { velocityThreshold_ = threshold; }
void ControlEngine::setMaxVelocity(double maxVel) { maxVelocity_ = maxVel; }
void ControlEngine::setHomePosition(const JointAngles& home) { homePosition_ = home; }

// ========================================
// 【生成随机点位】
// 为每个关节独立生成一个范围内的随机角度。
// 
// 算法：
//   随机角度 = 最小值 + random(0,1) × (最大值 - 最小值)
// 
// dist_(rng_) 会生成 0~1 之间的随机小数。
// ========================================
JointAngles ControlEngine::generateRandomPoint() {
    JointAngles point;
    
    for (int i = 0; i < 6; ++i) {
        // 计算当前关节的范围宽度
        double range = maxAngles_.j[i] - minAngles_.j[i];
        
        // 生成随机角度
        point.j[i] = minAngles_.j[i] + dist_(rng_) * range;
    }
    
    return point;
}

// ========================================
// 【等待机器人静止】
// 分两步：
//   1. 先"硬等" settlingTime_ 秒（给机械振动衰减的时间）
//   2. 再检查关节速度是否低于阈值（双重保险）
// ========================================
bool ControlEngine::waitForSettling() {
    // 第一步：固定等待时间
    std::cout << "  等待 " << settlingTime_ << " 秒让机器人稳定..." << std::endl;
    
    // std::chrono::milliseconds 把时间转换成毫秒
    // static_cast<int> 是 C++ 的类型转换：把 double 秒转成 int 毫秒
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(settlingTime_ * 1000))
    );

    // 第二步：检查速度（最多等 5 秒）
    auto t0 = std::chrono::steady_clock::now();  // 记录当前时间
    
    while (true) {
        // 调用接口检查是否静止
        if (robot_->isRobotStatic(velocityThreshold_)) {
            std::cout << "  机器人已静止（速度 < " << velocityThreshold_ 
                      << " rad/s）" << std::endl;
            return true;
        }
        
        // 计算已经等了多久
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - t0
        ).count();
        
        // 如果超过 5 秒还没静止，打印警告但继续执行
        if (elapsed > 5) {
            std::cerr << "  警告：等待静止超时，继续执行。" << std::endl;
            return true;
        }
        
        // 每 100 毫秒检查一次（避免 CPU 空转占用 100%）
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ========================================
// 【在当前位置采集数据】
// 采集关节角、关节扭矩、六维力、时间戳，然后写入 CSV。
// ========================================
bool ControlEngine::collectAtPoint(int pointIndex) {
    RobotDataSample sample;     // 创建一条空的采样记录
    
    sample.pointIndex = pointIndex;           // 记录点位编号
    sample.timestamp = robot_->getTimestamp(); // 获取时间戳
    
    // 依次调用接口获取数据（& 表示传引用，函数会把结果写进 sample 里）
    if (!robot_->getJointAngles(sample.angles)) return false;
    if (!robot_->getJointTorques(sample.torques)) return false;
    if (!robot_->getSixAxisForce(sample.force)) return false;
    
    // 把这条记录写入 CSV 文件
    if (!recorder_->record(sample)) return false;
    
    // 在屏幕上打印简要信息，方便观察进度
    std::cout << "  点位 " << pointIndex 
              << " | 时间=" << sample.timestamp
              << " | 关节角=[" << sample.angles.j[0]
              << "," << sample.angles.j[1]
              << "," << sample.angles.j[2]
              << "," << sample.angles.j[3]
              << "," << sample.angles.j[4]
              << "," << sample.angles.j[5] << "]" << std::endl;
    
    return true;
}

// ========================================
// 【回到 Home 位】
// ========================================
bool ControlEngine::returnToHome() {
    std::cout << "正在回到 Home 位..." << std::endl;
    return robot_->moveToJointPosition(homePosition_, maxVelocity_);
}

// ========================================
// 【主控流程】
// 这是整个程序最核心的函数，控制实验的完整流程。
// ========================================
bool ControlEngine::run() {
    std::cout << "\n========== 控制引擎启动 ==========" << std::endl;
    std::cout << "采集点数: " << pointCount_ 
              << " | 静止等待: " << settlingTime_ << "秒" << std::endl;

    // ===== 第 1 步：初始化高速数据通道 =====
    // 必须先初始化 EDG，否则拿不到关节扭矩！
    if (!robot_->initDataAcquisition()) {
        std::cerr << "错误：初始化数据采集通道失败！" << std::endl;
        return false;
    }

    // ===== 第 2 步：回到 Home 位并采集参考点（编号 0） =====
    if (!returnToHome()) return false;
    waitForSettling();
    collectAtPoint(0);  // 0 号点是 Home 位的参考数据

    // ===== 第 3 步：随机点位循环 =====
    // for 循环：从 1 到 pointCount_，逐个生成随机点
    for (int i = 1; i <= pointCount_; ++i) {
        // 生成一个随机目标角度
        JointAngles target = generateRandomPoint();
        
        std::cout << "\n--- 点位 " << i << "/" << pointCount_ << " ---" << std::endl;
        std::cout << "  目标角度: [" << target.j[0] << "," << target.j[1] << ","
                  << target.j[2] << "," << target.j[3] << "," << target.j[4]
                  << "," << target.j[5] << "]" << std::endl;

        // 命令机器人运动到目标位置
        // 如果运动失败（比如奇异点、限位），打印错误并跳过这个点
        if (!robot_->moveToJointPosition(target, maxVelocity_)) {
            std::cerr << "  运动失败，跳过此点。" << std::endl;
            continue;   // continue 表示跳过本次循环的剩余部分，进入下一次循环
        }
        
        // 等待机器人完全静止
        waitForSettling();
        
        // 采集数据
        collectAtPoint(i);
    }

    // ===== 第 4 步：实验结束，回到 Home 位 =====
    std::cout << "\n========== 采集完成，正在回 Home ==========" << std::endl;
    returnToHome();
    waitForSettling();
    
    // ===== 第 5 步：关闭 EDG 通道 =====
    robot_->deinitDataAcquisition();
    
    std::cout << "========== 控制引擎正常结束 ==========" << std::endl;
    return true;
}
