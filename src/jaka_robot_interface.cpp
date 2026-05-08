#include "jaka_robot_interface.h"
#include <iostream>
#include <cmath>

JakaRobotInterface::JakaRobotInterface()
    : connected_(false), edgInitialized_(false) {}

JakaRobotInterface::~JakaRobotInterface() {
    if (edgInitialized_) deinitDataAcquisition();
    if (connected_) {
        disableRobot();
        powerOff();
        disconnect();
    }
}

bool JakaRobotInterface::checkReturn(errno_t ret, const std::string& func) {
    if (ret != ERR_SUCC) {
        std::cerr << "[JAKA API Error] " << func << " failed, code: " << ret << std::endl;
        return false;
    }
    return true;
}

bool JakaRobotInterface::connect(const std::string& ip,
                                  const std::string& username,
                                  const std::string& password) {
    // V3 控制器使用 GRPC 通道（is_grpc = true）
    errno_t ret = robot_.login_in(ip.c_str(), true, username.c_str(), password.c_str());
    if (!checkReturn(ret, "login_in")) return false;
    connected_ = true;
    startTime_ = std::chrono::high_resolution_clock::now();
    std::cout << "[JakaRobotInterface] Connected to " << ip << std::endl;
    return true;
}

bool JakaRobotInterface::disconnect() {
    if (!connected_) return true;
    errno_t ret = robot_.login_out();
    connected_ = false;
    return checkReturn(ret, "login_out");
}

bool JakaRobotInterface::powerOn() {
    return checkReturn(robot_.power_on(), "power_on");
}

bool JakaRobotInterface::powerOff() {
    return checkReturn(robot_.power_off(), "power_off");
}

bool JakaRobotInterface::enableRobot() {
    return checkReturn(robot_.enable_robot(), "enable_robot");
}

bool JakaRobotInterface::disableRobot() {
    return checkReturn(robot_.disable_robot(), "disable_robot");
}

bool JakaRobotInterface::moveToJointPosition(const JointAngles& angles, double speedPercent) {
    JointValue jv;
    for (int i = 0; i < 6; ++i) jv.jVal[i] = angles.j[i];
    // ABS: 绝对运动；TRUE: 阻塞等待到位
    errno_t ret = robot_.joint_move(&jv, ABS, TRUE, speedPercent);
    return checkReturn(ret, "joint_move");
}

bool JakaRobotInterface::isRobotStatic(double velocityThreshold) {
    if (!edgInitialized_) return false;
    EDGState edgState;
    if (!checkReturn(robot_.edg_get_stat(&edgState), "edg_get_stat")) return false;
    for (int i = 0; i < 6; ++i) {
        if (std::abs(edgState.jointVel.jVel[i]) > velocityThreshold) return false;
    }
    return true;
}

bool JakaRobotInterface::getJointAngles(JointAngles& angles) {
    if (edgInitialized_) {
        EDGState edgState;
        if (!checkReturn(robot_.edg_get_stat(&edgState), "edg_get_stat")) return false;
        for (int i = 0; i < 6; ++i) angles.j[i] = edgState.jointVal.jVal[i];
        return true;
    } else {
        JointValue jv;
        if (!checkReturn(robot_.get_joint_position(&jv), "get_joint_position")) return false;
        for (int i = 0; i < 6; ++i) angles.j[i] = jv.jVal[i];
        return true;
    }
}

bool JakaRobotInterface::getJointTorques(JointTorques& torques) {
    // 关节力矩仅通过 EDG 高速通道获取 [^11^][^23^]
    if (!edgInitialized_) {
        std::cerr << "[JakaRobotInterface] EDG not initialized, cannot get joint torques" << std::endl;
        return false;
    }
    EDGState edgState;
    if (!checkReturn(robot_.edg_get_stat(&edgState), "edg_get_stat")) return false;
    for (int i = 0; i < 6; ++i) torques.t[i] = edgState.jointTorq.jtorq[i];
    return true;
}

bool JakaRobotInterface::getSixAxisForce(SixAxisForce& force) {
    // type=3: 去除重力与偏置后的实际数据；type=1: 兼容接口 [^8^]
    FTSensorDataStr data;
    errno_t ret = robot_.get_ft_sensor_data(0, 3, &data);
    if (!checkReturn(ret, "get_ft_sensor_data(type=3)")) {
        ret = robot_.get_ft_sensor_data(0, 1, &data);
        if (!checkReturn(ret, "get_ft_sensor_data(type=1)")) return false;
    }
    // 若 SDK 中字段名不同，请根据 jktypes.h 实际定义调整
    force.fx = data.fx;  force.fy = data.fy;  force.fz = data.fz;
    force.tx = data.tx;  force.ty = data.ty;  force.tz = data.tz;
    return true;
}

double JakaRobotInterface::getTimestamp() {
    // 优先使用 EDG 硬件时间戳 [^11^]
    if (edgInitialized_) {
        unsigned long int details[3];
        if (checkReturn(robot_.edg_stat_details(details), "edg_stat_details")) {
            // details[0]: 秒, details[1]: 纳秒
            return static_cast<double>(details[0]) + static_cast<double>(details[1]) * 1e-9;
        }
    }
    // 回退到系统相对时间
    auto now = std::chrono::high_resolution_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - startTime_).count();
    return ns * 1e-9;
}

bool JakaRobotInterface::initDataAcquisition(const std::string& localIp) {
    // mode=1: 启用 EDG 数据反馈，但不占用伺服模式，可与 joint_move 共存 [^23^]
    errno_t ret = robot_.edg_init(true, localIp.c_str(), 10010, 1);
    if (!checkReturn(ret, "edg_init")) return false;
    edgInitialized_ = true;
    std::cout << "[JakaRobotInterface] EDG data acquisition initialized (mode=1)" << std::endl;
    return true;
}

bool JakaRobotInterface::deinitDataAcquisition() {
    if (!edgInitialized_) return true;
    errno_t ret = robot_.edg_init(false);
    edgInitialized_ = false;
    return checkReturn(ret, "edg_init(false)");
}
