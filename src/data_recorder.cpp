#include "data_recorder.h"
#include <iostream>     // 打印信息
#include <iomanip>      // 格式化输出（比如控制小数位数）

// ========================================
// 【构造函数】
// 只保存文件名，此时还不打开文件。
// ========================================
DataRecorder::DataRecorder(const std::string& filename)
    : filename_(filename), isOpen_(false), sampleCount_(0) {}

// ========================================
// 【析构函数】
// 对象销毁时自动关闭文件，防止数据丢失。
// ========================================
DataRecorder::~DataRecorder() {
    close();
}

// ========================================
// 【打开文件】
// std::ios::out：以输出模式打开（写入）
// std::ios::trunc：如果文件已存在，清空它（覆盖旧数据）
// ========================================
bool DataRecorder::open() {
    file_.open(filename_, std::ios::out | std::ios::trunc);
    
    if (!file_.is_open()) {
        std::cerr << "[DataRecorder] 错误：无法打开文件 " << filename_ << std::endl;
        return false;
    }
    
    isOpen_ = true;
    writeHeader();  // 写入 CSV 表头
    
    std::cout << "[DataRecorder] 已打开文件: " << filename_ << std::endl;
    return true;
}

// ========================================
// 【关闭文件】
// flush()：把内存缓冲区里的数据强制写入硬盘。
// 如果不 flush，操作系统可能暂时把数据存在内存里，断电会丢失。
// ========================================
bool DataRecorder::close() {
    if (isOpen_) {
        flush();
        file_.close();
        isOpen_ = false;
        std::cout << "[DataRecorder] 已关闭文件。共记录 " << sampleCount_ << " 条数据。" << std::endl;
    }
    return true;
}

// ========================================
// 【写入 CSV 表头】
// 第一行文字，说明每列代表什么含义。
// 用逗号分隔，方便 Excel 识别为不同列。
// ========================================
void DataRecorder::writeHeader() {
    file_ << "点位编号,时间戳,"
          << "关节1,关节2,关节3,关节4,关节5,关节6,"
          << "扭矩1,扭矩2,扭矩3,扭矩4,扭矩5,扭矩6,"
          << "力x,力y,力z,力矩x,力矩y,力矩z\n";
    
    file_.flush();  // 立刻写入硬盘
}

// ========================================
// 【记录一条数据】
// 把 RobotDataSample 结构体里的所有字段写成 CSV 的一行。
// 
// std::fixed：固定小数表示法（不用科学计数法）
// std::setprecision(6)：保留 6 位小数
// ========================================
bool DataRecorder::record(const RobotDataSample& sample) {
    if (!isOpen_) return false;  // 文件没打开，无法写入
    
    // 点位编号和时间戳
    file_ << sample.pointIndex << ",";
    file_ << std::fixed << std::setprecision(6) << sample.timestamp << ",";

    // 6 个关节角度（用循环减少重复代码）
    for (int i = 0; i < 6; ++i) {
        file_ << sample.angles.j[i];
        // 前 5 个后面加逗号，第 6 个不加（为了格式对齐，这里统一加逗号，后面继续）
        file_ << ",";
    }
    
    // 6 个关节扭矩
    for (int i = 0; i < 6; ++i) {
        file_ << sample.torques.t[i] << ",";
    }
    
    // 6 维力/力矩
    file_ << sample.force.fx << "," << sample.force.fy << "," << sample.force.fz << ","
          << sample.force.tx << "," << sample.force.ty << "," << sample.force.tz << "\n";

    ++sampleCount_;  // 计数器加 1
    
    // 每记录 10 条就 flush 一次，平衡性能和安全性
    if (sampleCount_ % 10 == 0) {
        file_.flush();
    }
    
    return true;
}

// ========================================
// 【强制刷新】
// 把操作系统缓冲区的数据立刻写入硬盘。
// ========================================
bool DataRecorder::flush() {
    if (isOpen_) {
        file_.flush();
    }
    return true;
}
