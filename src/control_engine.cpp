#include "control_engine.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

ControlEngine::ControlEngine(IRobotInterface* robot, DataRecorder* recorder)
    : robot_(robot), recorder_(recorder),
      pointCount_(50), settlingTime_(2.0),
      velocityThreshold_(0.001), maxVelocity_(20.0),
      rng_(std::random_device{}()), dist_(0.0, 1.0)
{
    // 默认关节限位（单位：弧度），请根据实际机型修改
    minAngles_ = JointAngles(-2.967, -1.745, -2.967, -3.316, -2.967, -3.316);
    maxAngles_ = JointAngles( 2.967,  1.745,  2.967,  3.316,  2.967,  3.316);
    homePosition_ = JointAngles(0, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0);
}

void ControlEngine::setJointLimits(const JointAngles& minAngles, const JointAngles& maxAngles) {
    minAngles_ = minAngles; maxAngles_ = maxAngles;
}
void ControlEngine::setPointCount(int count) { pointCount_ = count; }
void ControlEngine::setSettlingTime(double seconds) { settlingTime_ = seconds; }
void ControlEngine::setVelocityThreshold(double threshold) { velocityThreshold_ = threshold; }
void ControlEngine::setMaxVelocity(double maxVel) { maxVelocity_ = maxVel; }
void ControlEngine::setHomePosition(const JointAngles& home) { homePosition_ = home; }

JointAngles ControlEngine::generateRandomPoint() {
    JointAngles point;
    for (int i = 0; i < 6; ++i) {
        point.j[i] = minAngles_.j[i] + dist_(rng_) * (maxAngles_.j[i] - minAngles_.j[i]);
    }
    return point;
}

bool ControlEngine::waitForSettling() {
    std::cout << "  Waiting " << settlingTime_ << "s for settling..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(settlingTime_ * 1000)));

    // 额外检查关节速度是否低于阈值
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        if (robot_->isRobotStatic(velocityThreshold_)) {
            std::cout << "  Robot is static (vel < " << velocityThreshold_ << " rad/s)." << std::endl;
            return true;
        }
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (elapsed > 5) {
            std::cerr << "  Warning: timeout waiting for static, proceeding." << std::endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool ControlEngine::collectAtPoint(int pointIndex) {
    RobotDataSample sample;
    sample.pointIndex = pointIndex;
    sample.timestamp = robot_->getTimestamp();

    if (!robot_->getJointAngles(sample.angles)) return false;
    if (!robot_->getJointTorques(sample.torques)) return false;
    if (!robot_->getSixAxisForce(sample.force)) return false;

    if (!recorder_->record(sample)) return false;

    std::cout << "  Point " << pointIndex << " | t=" << sample.timestamp
              << " | J=[" << sample.angles.j[0]
              << "," << sample.angles.j[1]
              << "," << sample.angles.j[2]
              << "," << sample.angles.j[3]
              << "," << sample.angles.j[4]
              << "," << sample.angles.j[5] << "]" << std::endl;
    return true;
}

bool ControlEngine::returnToHome() {
    std::cout << "Returning to home..." << std::endl;
    return robot_->moveToJointPosition(homePosition_, maxVelocity_);
}

bool ControlEngine::run() {
    std::cout << "=== Control Engine Start ===" << std::endl;
    std::cout << "Points: " << pointCount_ << ", Settling: " << settlingTime_ << "s" << std::endl;

    if (!robot_->initDataAcquisition()) {
        std::cerr << "Failed to init data acquisition." << std::endl;
        return false;
    }

    // 回到 Home 并采集参考点
    if (!returnToHome()) return false;
    waitForSettling();
    collectAtPoint(0);

    // 随机点位循环
    for (int i = 1; i <= pointCount_; ++i) {
        JointAngles target = generateRandomPoint();
        std::cout << "\nPoint " << i << "/" << pointCount_ << std::endl;
        std::cout << "  Target: [" << target.j[0] << "," << target.j[1] << ","
                  << target.j[2] << "," << target.j[3] << "," << target.j[4]
                  << "," << target.j[5] << "]" << std::endl;

        if (!robot_->moveToJointPosition(target, maxVelocity_)) {
            std::cerr << "  Move failed, skipping." << std::endl;
            continue;
        }
        waitForSettling();
        collectAtPoint(i);
    }

    std::cout << "\n=== Completed. Returning home ===" << std::endl;
    returnToHome();
    waitForSettling();
    robot_->deinitDataAcquisition();
    std::cout << "=== Control Engine Finished ===" << std::endl;
    return true;
}
