#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "types.h"      // 引入上面定义的数据结构
#include <string>       // 引入字符串类，用于 IP 地址、用户名等

// ========================================
// 【纯虚类 / 接口类】
// 
// 在 C++ 中，"class" 是定义"类"的关键字。类就像一张"设计图纸"，
// 描述了对象应该有什么功能。
// 
// "virtual" 表示"虚函数"：意思是"子类必须自己实现这个功能"。
// "= 0" 表示这是一个"纯虚函数"，强制要求子类必须重写。
// 
// 这个类的目的是：给控制算法层提供一个"统一的操作机器人的说明书"，
// 控制层不需要知道底层是 JAKA 机器人还是别的品牌，只需要按这个说明书操作。
// ========================================

class IRobotInterface {
public:
    // 【虚析构函数】
    // 当用父类指针删除子类对象时，确保子类的析构函数也能被正确调用
    // "default" 表示使用编译器自动生成的默认实现
    virtual ~IRobotInterface() = default;

    // ================== 连接管理 ==================
    
    // 连接机器人
    // const std::string& 表示"字符串的常量引用"：只读、不拷贝、效率高
    // 返回值 bool：true 表示成功，false 表示失败
    virtual bool connect(const std::string& ip,
                         const std::string& username = "jaka_sdk",
                         const std::string& password = "") = 0;
    
    // 断开连接
    virtual bool disconnect() = 0;

    // ================== 电源与使能 ==================
    
    // 上电：给机器人电机供电（相当于按下电源开关）
    virtual bool powerOn() = 0;
    
    // 下电：切断电机供电
    virtual bool powerOff() = 0;
    
    // 使能机器人：解除抱闸，电机可以运动（相当于松开手刹）
    virtual bool enableRobot() = 0;
    
    // 禁用机器人：抱闸锁死，电机不能运动（相当于拉手刹）
    virtual bool disableRobot() = 0;

    // ================== 运动控制 ==================
    
    // 移动到指定的关节角度位置
    // speedPercent：运动速度百分比（比如 20 表示 20% 最大速度）
    virtual bool moveToJointPosition(const JointAngles& angles, double speedPercent = 20.0) = 0;
    
    // 判断机器人是否静止
    // velocityThreshold：速度阈值，低于此值认为已静止
    virtual bool isRobotStatic(double velocityThreshold = 0.001) = 0;

    // ================== 数据采集 ==================
    
    // 获取当前 6 个关节的角度（结果写入 angles 引用中）
    virtual bool getJointAngles(JointAngles& angles) = 0;
    
    // 获取当前 6 个关节的扭矩
    virtual bool getJointTorques(JointTorques& torques) = 0;
    
    // 获取末端六维力/力矩
    virtual bool getSixAxisForce(SixAxisForce& force) = 0;
    
    // 获取当前时间戳（秒）
    virtual double getTimestamp() = 0;

    // ================== 高速数据通道 ==================
    
    // 初始化数据采集通道（EDG：External Data Gateway）
    // localIp：本机 IP 地址，用于接收机器人高速推送的数据
    virtual bool initDataAcquisition(const std::string& localIp = "0.0.0.0") = 0;
    
    // 关闭数据采集通道
    virtual bool deinitDataAcquisition() = 0;
};

#endif
