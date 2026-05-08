#ifndef JAKA_ROBOT_INTERFACE_H
#define JAKA_ROBOT_INTERFACE_H

#include "robot_interface.h"
#include "JAKAZuRobot.h"
#include <chrono>

class JakaRobotInterface : public IRobotInterface {
public:
    JakaRobotInterface();
    ~JakaRobotInterface() override;

    bool connect(const std::string& ip,
                 const std::string& username = "jaka_sdk",
                 const std::string& password = "") override;
    bool disconnect() override;

    bool powerOn() override;
    bool powerOff() override;
    bool enableRobot() override;
    bool disableRobot() override;

    bool moveToJointPosition(const JointAngles& angles, double speedPercent = 20.0) override;
    bool isRobotStatic(double velocityThreshold = 0.001) override;

    bool getJointAngles(JointAngles& angles) override;
    bool getJointTorques(JointTorques& torques) override;
    bool getSixAxisForce(SixAxisForce& force) override;
    double getTimestamp() override;

    bool initDataAcquisition(const std::string& localIp = "0.0.0.0") override;
    bool deinitDataAcquisition() override;

private:
    JAKAZuRobot robot_;
    bool connected_;
    bool edgInitialized_;
    std::chrono::high_resolution_clock::time_point startTime_;

    bool checkReturn(errno_t ret, const std::string& func);
};

#endif
