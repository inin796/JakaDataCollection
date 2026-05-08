// 引入自己的头文件
#include "jaka_robot_interface.h"

// 引入标准库
#include <iostream>     // 打印错误信息
#include <cmath>        // 数学库，提供 abs() 绝对值函数

// ========================================
// 【构造函数】
// 对象诞生时执行，初始化成员变量。
// 冒号后面的 ": connected_(false), edgInitialized_(false)" 叫"初始化列表"，
// 是 C++ 给成员变量赋初始值的推荐方式，效率更高。
// ========================================
JakaRobotInterface::JakaRobotInterface()
    : connected_(false), edgInitialized_(false) {}

// ========================================
// 【析构函数】
// 对象死亡时（比如程序结束、delete 时）自动执行。
// 这里做"善后工作"：关闭 EDG、禁用机器人、断电、断开连接。
// 即使程序异常退出，这些操作也会被执行，防止机器人处于危险状态。
// ========================================
JakaRobotInterface::~JakaRobotInterface() {
    // 如果 EDG 还开着，先关闭
    if (edgInitialized_) {
        deinitDataAcquisition();
    }
    
    // 如果还连着机器人，安全关闭
    if (connected_) {
        disableRobot();     // 禁用（抱闸）
        powerOff();         // 断电
        disconnect();       // 断网
    }
}

// ========================================
// 【辅助函数】检查 SDK 返回值
// JAKA SDK 的每个函数都返回 errno_t 类型的错误码。
// ERR_SUCC（值为 0）表示成功，其他值表示各种错误。
// ========================================
bool JakaRobotInterface::checkReturn(errno_t ret, const std::string& func) {
    if (ret != ERR_SUCC) {
        // std::cerr 是"标准错误输出流"，用于打印错误信息（红色字体）
        std::cerr << "[JAKA API 错误] 函数 " << func 
                  << " 调用失败，错误码：" << ret << std::endl;
        return false;   // 返回 false 表示出错了
    }
    return true;        // 返回 true 表示成功
}

// ========================================
// 【连接机器人】
// login_in() 是 JAKA SDK 的登录/连接函数。
// 参数：
//   ip.c_str()：把 C++ string 转成 C 语言风格的字符数组（const char*）
//   true：使用 GRPC 协议（V3 控制器必须用这个）
//   username.c_str()：用户名，固定为 "jaka_sdk"
//   password.c_str()：密码，V3 需要在 APP 里设置
// ========================================
bool JakaRobotInterface::connect(const std::string& ip,
                                  const std::string& username,
                                  const std::string& password) {
    errno_t ret = robot_.login_in(ip.c_str(), true, username.c_str(), password.c_str());
    
    if (!checkReturn(ret, "login_in")) {
        return false;   // 连接失败
    }
    
    connected_ = true;
    
    // 记录程序启动时间，用于计算相对时间戳
    startTime_ = std::chrono::high_resolution_clock::now();
    
    std::cout << "[JakaRobotInterface] 已连接到 " << ip << std::endl;
    return true;
}

// ========================================
// 【断开连接】
// login_out() 是 SDK 的登出函数。
// ========================================
bool JakaRobotInterface::disconnect() {
    if (!connected_) return true;   // 如果本来就没连，直接返回成功
    
    errno_t ret = robot_.login_out();
    connected_ = false;
    return checkReturn(ret, "login_out");
}

// ========================================
// 【电源控制】
// 直接调用 SDK 函数，并用 checkReturn 检查是否成功。
// ========================================
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

// ========================================
// 【运动到指定关节角度】
// JointValue 是 JAKA SDK 定义的结构体，里面有一个 jVal[6] 数组。
// 我们需要把自定义的 JointAngles 转换过去。
// 
// ABS 表示"绝对运动"（目标值就是绝对角度，不是相对偏移）。
// TRUE 表示"阻塞模式"：函数会等机器人运动到位后才返回。
// speedPercent：速度百分比。
// ========================================
bool JakaRobotInterface::moveToJointPosition(const JointAngles& angles, double speedPercent) {
    JointValue jv;  // JAKA SDK 的关节角度结构体
    
    // 把自定义的 angles.j[0]~j[5] 拷贝到 SDK 的 jv.jVal[0]~jVal[5]
    for (int i = 0; i < 6; ++i) {
        jv.jVal[i] = angles.j[i];
    }
    
    // 调用 SDK 运动函数
    errno_t ret = robot_.joint_move(&jv, ABS, TRUE, speedPercent);
    return checkReturn(ret, "joint_move");
}

// ========================================
// 【判断机器人是否静止】
// 通过 EDG 高速通道获取实时关节速度。
// 如果 6 个关节的速度都小于阈值，就认为静止了。
// ========================================
bool JakaRobotInterface::isRobotStatic(double velocityThreshold) {
    // 如果 EDG 没初始化，无法获取速度，返回 false
    if (!edgInitialized_) return false;
    
    EDGState edgState;  // EDG 状态结构体，包含速度、力矩等实时数据
    
    // 获取 EDG 状态
    if (!checkReturn(robot_.edg_get_stat(&edgState), "edg_get_stat")) {
        return false;
    }
    
    // 遍历 6 个关节
    for (int i = 0; i < 6; ++i) {
        // std::abs() 取绝对值
        // 如果任何一个关节速度超过阈值，就认为还没静止
        if (std::abs(edgState.jointVel.jVel[i]) > velocityThreshold) {
            return false;
        }
    }
    
    // 所有关节速度都低于阈值，返回 true（已静止）
    return true;
}

// ========================================
// 【获取关节角度】
// 优先使用 EDG 高速通道（频率高、延迟低），
// 如果 EDG 没开，就退回到普通查询接口（频率低一些）。
// ========================================
bool JakaRobotInterface::getJointAngles(JointAngles& angles) {
    if (edgInitialized_) {
        // ===== 使用 EDG 高速通道 =====
        EDGState edgState;
        if (!checkReturn(robot_.edg_get_stat(&edgState), "edg_get_stat")) {
            return false;
        }
        for (int i = 0; i < 6; ++i) {
            angles.j[i] = edgState.jointVal.jVal[i];
        }
        return true;
    } else {
        // ===== 退回到普通接口 =====
        JointValue jv;
        if (!checkReturn(robot_.get_joint_position(&jv), "get_joint_position")) {
            return false;
        }
        for (int i = 0; i < 6; ++i) {
            angles.j[i] = jv.jVal[i];
        }
        return true;
    }
}

// ========================================
// 【获取关节扭矩】
// 关节扭矩只能通过 EDG 高速通道获取！普通查询接口没有这个功能。
// 所以必须先调用 initDataAcquisition() 初始化 EDG。
// ========================================
bool JakaRobotInterface::getJointTorques(JointTorques& torques) {
    if (!edgInitialized_) {
        std::cerr << "[JakaRobotInterface] 错误：EDG 未初始化，无法获取关节扭矩！" << std::endl;
        return false;
    }
    
    EDGState edgState;
    if (!checkReturn(robot_.edg_get_stat(&edgState), "edg_get_stat")) {
        return false;
    }
    
    // edgState.jointTorq.jtorq[i] 是第 i 个关节的扭矩
    for (int i = 0; i < 6; ++i) {
        torques.t[i] = edgState.jointTorq.jtorq[i];
    }
    return true;
}

// ========================================
// 【获取六维力/力矩】
// get_ft_sensor_data() 是力传感器接口。
// 参数：
//   0：传感器编号（单传感器固定为 0）
//   3：数据类型（3=去除重力和偏置后的实际数据，最常用）
//   &data：结果写入这个结构体
// 
// 如果 type=3 失败（老版本控制器可能不支持），就尝试 type=1。
// ========================================
bool JakaRobotInterface::getSixAxisForce(SixAxisForce& force) {
    FTSensorDataStr data;
    
    // 先尝试 type=3（推荐）
    errno_t ret = robot_.get_ft_sensor_data(0, 3, &data);
    
    if (!checkReturn(ret, "get_ft_sensor_data(type=3)")) {
        // type=3 失败了，尝试 type=1（兼容模式）
        ret = robot_.get_ft_sensor_data(0, 1, &data);
        if (!checkReturn(ret, "get_ft_sensor_data(type=1)")) {
            return false;
        }
    }
    
    // 把 SDK 结构体的字段拷贝到我们的自定义结构体
    // 注意：如果 SDK 头文件里的字段名不同，请根据实际定义调整
    force.fx = data.fx;
    force.fy = data.fy;
    force.fz = data.fz;
    force.tx = data.tx;
    force.ty = data.ty;
    force.tz = data.tz;
    
    return true;
}

// ========================================
// 【获取时间戳】
// 优先使用 EDG 硬件时间戳（与机器人控制器时钟同步，精度最高）。
// 如果 EDG 不可用，就使用电脑的系统时间（相对时间）。
// ========================================
double JakaRobotInterface::getTimestamp() {
    // ===== 尝试 EDG 硬件时间戳 =====
    if (edgInitialized_) {
        unsigned long int details[3];  // details[0]=秒, details[1]=纳秒
        
        if (checkReturn(robot_.edg_stat_details(details), "edg_stat_details")) {
            // 把秒和纳秒合并成一个 double（单位：秒）
            // 1 纳秒 = 1e-9 秒
            return static_cast<double>(details[0]) 
                 + static_cast<double>(details[1]) * 1e-9;
        }
    }
    
    // ===== 退回到系统相对时间 =====
    auto now = std::chrono::high_resolution_clock::now();
    
    // 计算从 startTime_ 到现在经过了多少纳秒
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - startTime_).count();
    
    // 纳秒转秒
    return ns * 1e-9;
}

// ========================================
// 【初始化数据采集通道（EDG）】
// edg_init() 开启高速数据反馈通道。
// 参数：
//   true：启用
//   localIp.c_str()：本机 IP 地址（机器人会把数据推送到这个 IP）
//   10010：端口号
//   1：模式 1（启用数据反馈，但不占用伺服模式，可与 joint_move 共存）
// ========================================
bool JakaRobotInterface::initDataAcquisition(const std::string& localIp) {
    errno_t ret = robot_.edg_init(true, localIp.c_str(), 10010, 1);
    
    if (!checkReturn(ret, "edg_init")) {
        return false;
    }
    
    edgInitialized_ = true;
    std::cout << "[JakaRobotInterface] EDG 数据采集通道已初始化（模式=1）" << std::endl;
    return true;
}

// ========================================
// 【关闭 EDG 通道】
// edg_init(false) 表示关闭。
// ========================================
bool JakaRobotInterface::deinitDataAcquisition() {
    if (!edgInitialized_) return true;   // 本来就没开，直接返回
    
    errno_t ret = robot_.edg_init(false);
    edgInitialized_ = false;
    return checkReturn(ret, "edg_init(false)");
}
