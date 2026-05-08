// 引入自定义头文件
#include "jaka_robot_interface.h"   // JAKA 接口实现
#include "control_engine.h"         // 控制引擎
#include "data_recorder.h"          // 数据记录器

// 引入标准库
#include <iostream>     // 输入输出流：用于打印信息到屏幕（cout/cerr）
#include <csignal>      // 信号处理：用于捕获 Ctrl+C 等中断信号
#include <thread>       // 线程库：用于让程序"睡觉"（休眠等待）
#include <chrono>       // 时间库：提供秒、毫秒等时间单位

// ========================================
// 【全局变量】
// 全局变量在整个程序的任何地方都能访问。
// g_running 用来标记程序是否还在运行，收到中断信号后设为 false。
// ========================================
static bool g_running = true;

// 【信号处理函数】
// 当用户按下 Ctrl+C 时，操作系统会发送 SIGINT 信号给程序，
// 这个函数就会被调用，把 g_running 设为 false，让程序优雅退出。
void signalHandler(int signum) {
    std::cout << "\n收到信号 " << signum << "，正在停止程序..." << std::endl;
    g_running = false;
}

// ========================================
// 【main 函数】：程序的入口，就像 C 语言程序的"大门"
// argc：命令行参数的数量（argument count）
// argv：命令行参数的字符串数组（argument vector）
//   argv[0] 是程序名
//   argv[1] 是第一个参数（机器人 IP）
//   argv[2] 是第二个参数（点位数量）
//   argv[3] 是第三个参数（CSV 文件名）
// ========================================
int main(int argc, char* argv[]) {
    // 注册信号处理函数：捕获 Ctrl+C（SIGINT）
    signal(SIGINT, signalHandler);

    // ================== 默认参数 ==================
    std::string robotIp = "192.168.2.194";  // 机器人默认 IP
    std::string password = "";              // V3 控制器需要在 APP 设置密码
    std::string csvFile = "robot_data.csv"; // 默认输出文件名
    int pointCount = 50;                    // 默认采集 50 个随机点

    // ================== 解析命令行参数 ==================
    // 如果用户在命令行输入了参数，就用用户的，否则用默认值
    if (argc > 1) robotIp = argv[1];        // 第 1 个参数：IP 地址
    if (argc > 2) pointCount = std::stoi(argv[2]);  // 第 2 个参数：点数（字符串转整数）
    if (argc > 3) csvFile = argv[3];        // 第 3 个参数：文件名

    // 打印实验信息
    std::cout << "===== JAKA 随机点位数据采集程序 =====" << std::endl;
    std::cout << "机器人 IP: " << robotIp << std::endl;
    std::cout << "采集点数: " << pointCount << std::endl;
    std::cout << "输出文件: " << csvFile << std::endl;

    // ================== 创建对象 ==================
    // 第 1 步：创建机器人接口对象（实际类型是 JakaRobotInterface）
    JakaRobotInterface robot;
    
    // 第 2 步：创建数据记录器对象
    DataRecorder recorder(csvFile);
    
    // 第 3 步：创建控制引擎，把上面两个对象的"地址"传给它
    // "&" 是取地址符：&robot 表示"robot 对象在内存中的位置"
    // 控制引擎通过指针操作这两个对象，这就是"多态"的核心
    ControlEngine engine(&robot, &recorder);

    // ================== 配置实验参数 ==================
    // 以下角度单位都是【弧度】（rad），不是角度（°）！
    // π rad = 180°，所以 2.967 rad ≈ 170°
    
    // 设置关节运动范围（安全限位，防止撞到东西）
    // 请根据你的实际机械臂型号和工作空间修改！
    engine.setJointLimits(
        JointAngles(-2.967, -1.745, -2.967, -3.316, -2.967, -3.316),  // 最小角度
        JointAngles( 2.967,  1.745,  2.967,  3.316,  2.967,  3.316)   // 最大角度
    );
    
    // 设置 Home 位（程序开始和结束的位置）
    // M_PI 是 C++ 中 π 的常量（约 3.14159）
    engine.setHomePosition(JointAngles(0, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0));
    
    engine.setPointCount(pointCount);       // 随机点位数量
    engine.setSettlingTime(2.0);            // 到位后等待 2 秒，确保静止
    engine.setVelocityThreshold(0.001);     // 关节速度 < 0.001 rad/s 才认为是静止
    engine.setMaxVelocity(20.0);            // 运动速度 20%（较慢，保证安全）

    // ================== 打开 CSV 文件 ==================
    // 如果文件打开失败，返回 -1 表示程序异常退出
    if (!recorder.open()) {
        std::cerr << "错误：无法打开 CSV 文件！" << std::endl;
        return -1;
    }

    // ================== 连接并启动机器人 ==================
    // connect()：通过网络连接到机器人控制器
    // 参数：IP 地址、用户名（固定为 jaka_sdk）、密码
    if (!robot.connect(robotIp, "jaka_sdk", password)) {
        std::cerr << "错误：连接机器人失败！" << std::endl;
        return -1;
    }

    // powerOn()：给电机上电（会有"咔哒"一声，抱闸松开）
    if (!robot.powerOn()) {
        std::cerr << "错误：机器人上电失败！" << std::endl;
        return -1;
    }
    
    // sleep_for：让当前线程休眠一段时间
    // 上电后等待 2 秒，让系统稳定
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // enableRobot()：使能机器人（电机可以接收运动指令）
    if (!robot.enableRobot()) {
        std::cerr << "错误：使能机器人失败！" << std::endl;
        return -1;
    }
    
    // 使能后再等 1 秒
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ================== 执行主流程 ==================
    // run() 是控制引擎的核心函数，执行：
    //   1. 初始化 EDG 数据采集
    //   2. 回到 Home 位
    //   3. 循环：生成随机点 → 运动 → 等待静止 → 采集数据
    //   4. 回到 Home 位
    //   5. 关闭 EDG
    bool result = engine.run();

    // ================== 收尾工作 ==================
    // 无论成功还是失败，都要安全关闭机器人
    
    // disableRobot()：禁用机器人（电机抱闸锁死）
    robot.disableRobot();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 等 0.5 秒
    
    // powerOff()：切断电机电源
    robot.powerOff();
    
    // disconnect()：断开网络连接
    robot.disconnect();
    
    // close()：关闭 CSV 文件
    recorder.close();

    // ================== 返回结果 ==================
    // result 为 true 返回 0（Linux 惯例：0 表示成功）
    // result 为 false 返回 -1（表示出错）
    std::cout << "程序结束，结果：" << (result ? "成功" : "失败") << std::endl;
    return result ? 0 : -1;
}
