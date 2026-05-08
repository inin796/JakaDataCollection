#include "jaka_robot_interface.h"
#include "control_engine.h"
#include "data_recorder.h"
#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

static bool g_running = true;
void signalHandler(int signum) {
    std::cout << "\nSignal " << signum << " received. Stopping..." << std::endl;
    g_running = false;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    // 默认参数
    std::string robotIp = "192.168.2.194";
    std::string password = "";          // V3 需在 APP 中设置 jaka_sdk 密码 [^6^]
    std::string csvFile = "robot_data.csv";
    int pointCount = 50;

    if (argc > 1) robotIp = argv[1];
    if (argc > 2) pointCount = std::stoi(argv[2]);
    if (argc > 3) csvFile = argv[3];

    std::cout << "JAKA Random Point Data Collection" << std::endl;
    std::cout << "IP: " << robotIp << " | Points: " << pointCount << " | CSV: " << csvFile << std::endl;

    JakaRobotInterface robot;
    DataRecorder recorder(csvFile);
    ControlEngine engine(&robot, &recorder);

    // 配置：请根据实际机型修改关节限位（单位：弧度）
    engine.setJointLimits(
        JointAngles(-2.967, -1.745, -2.967, -3.316, -2.967, -3.316),
        JointAngles( 2.967,  1.745,  2.967,  3.316,  2.967,  3.316)
    );
    engine.setHomePosition(JointAngles(0, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0));
    engine.setPointCount(pointCount);
    engine.setSettlingTime(2.0);          // 到位后静止 2 秒
    engine.setVelocityThreshold(0.001);    // 关节速度 < 0.001 rad/s 视为静止
    engine.setMaxVelocity(20.0);           // 运动速度 20%

    if (!recorder.open()) return -1;

    if (!robot.connect(robotIp, "jaka_sdk", password)) return -1;
    if (!robot.powerOn()) return -1;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    if (!robot.enableRobot()) return -1;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    bool result = engine.run();

    robot.disableRobot();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot.powerOff();
    robot.disconnect();
    recorder.close();

    return result ? 0 : -1;
}
