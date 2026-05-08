#ifndef CONTROL_ENGINE_H
#define CONTROL_ENGINE_H

#include "types.h"
#include "robot_interface.h"
#include "data_recorder.h"
#include <random>

class ControlEngine {
public:
    ControlEngine(IRobotInterface* robot, DataRecorder* recorder);

    void setJointLimits(const JointAngles& minAngles, const JointAngles& maxAngles);
    void setPointCount(int count);
    void setSettlingTime(double seconds);
    void setVelocityThreshold(double threshold);
    void setMaxVelocity(double maxVel);
    void setHomePosition(const JointAngles& home);

    bool run();

private:
    IRobotInterface* robot_;
    DataRecorder* recorder_;

    JointAngles minAngles_, maxAngles_, homePosition_;
    int pointCount_;
    double settlingTime_;
    double velocityThreshold_;
    double maxVelocity_;

    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;

    JointAngles generateRandomPoint();
    bool waitForSettling();
    bool collectAtPoint(int pointIndex);
    bool returnToHome();
};

#endif
