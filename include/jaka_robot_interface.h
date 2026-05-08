#ifndef JAKA_ROBOT_INTERFACE_H
#define JAKA_ROBOT_INTERFACE_H

#include "robot_interface.h"    // 引入抽象接口（上面定义的"说明书"）
#include "JAKAZuRobot.h"        // JAKA 官方 SDK 的头文件（只有这里包含它！）
#include <chrono>               // C++ 时间库，用于计算时间戳

// ========================================
// 【继承】：JakaRobotInterface 继承了 IRobotInterface
// 
// "public IRobotInterface" 表示：JakaRobotInterface 承诺会实现父类中所有纯虚函数。
// 这就像儿子继承了父亲的基因，但必须自己学会走路、说话（实现函数）。
// 
// 这个类是"唯一"直接调用 JAKA API 的地方。
// 控制算法层看不到这个类，只能看到父类 IRobotInterface。
// ========================================

class JakaRobotInterface : public IRobotInterface {
public:
    // 构造函数：创建对象时执行
    JakaRobotInterface();
    
    // 析构函数：对象销毁时自动执行（比如程序结束、delete 时）
    // override 表示"这是重写父类的虚函数"，如果父类没有对应函数会编译报错
    ~JakaRobotInterface() override;

    // 以下所有函数都带 "override"，表示它们是对父类纯虚函数的具体实现
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
    // 【成员变量】：这个对象内部保存的数据
    
    JAKAZuRobot robot_;     // JAKA SDK 的核心对象，所有 API 都通过它调用
    bool connected_;        // 标记是否已连接（true=已连接，false=未连接）
    bool edgInitialized_;   // 标记 EDG 高速通道是否已初始化
    
    // 程序启动时间点，用于计算相对时间戳
    std::chrono::high_resolution_clock::time_point startTime_;

    // 【私有辅助函数】：只在类内部使用，外部看不到
    // 检查 SDK 返回值，如果出错就打印错误信息
    bool checkReturn(errno_t ret, const std::string& func);
};

#endif
