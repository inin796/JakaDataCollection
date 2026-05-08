#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "types.h"
#include <string>

class IRobotInterface {
public:
    virtual ~IRobotInterface() = default;

    // 连接管理
    virtual bool connect(const std::string& ip,
                         const std::string& username = "jaka_sdk",
                         const std::string& password = "") = 0;
    virtual bool disconnect() = 0;

    // 电源与使能
    virtual bool powerOn() = 0;
    virtual bool powerOff() = 0;
    virtual bool enableRobot() = 0;
    virtual bool disableRobot() = 0;

    // 运动控制
    virtual bool moveToJointPosition(const JointAngles& angles, double speedPercent = 20.0) = 0;
    virtual bool isRobotStatic(double velocityThreshold = 0.001) = 0;

    // 数据采集
    virtual bool getJointAngles(JointAngles& angles) = 0;
    virtual bool getJointTorques(JointTorques& torques) = 0;
    virtual bool getSixAxisForce(SixAxisForce& force) = 0;
    virtual double getTimestamp() = 0;

    // 高速数据通道初始化（EDG）
    virtual bool initDataAcquisition(const std::string& localIp = "0.0.0.0") = 0;
    virtual bool deinitDataAcquisition() = 0;
};

#endif
