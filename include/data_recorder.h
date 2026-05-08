#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H

#include "types.h"      // 需要用到 RobotDataSample 结构体
#include <string>       // 文件名用字符串
#include <fstream>      // C++ 文件流：用于读写文件

// ========================================
// 【数据记录器】：负责把采集到的数据写入 CSV 文件
// 
// CSV（Comma-Separated Values）是一种纯文本表格格式，
// 用逗号分隔列，用换行分隔行，可以直接用 Excel 打开。
// ========================================

class DataRecorder {
public:
    // 构造函数：传入要保存的文件名（如 "robot_data.csv"）
    explicit DataRecorder(const std::string& filename);
    
    // 析构函数：对象销毁时自动关闭文件
    ~DataRecorder();

    // 打开文件并写入表头
    bool open();
    
    // 关闭文件
    bool close();
    
    // 记录一条数据（写入 CSV 的一行）
    bool record(const RobotDataSample& sample);
    
    // 强制刷新缓冲区（确保数据立刻写入硬盘，而不是留在内存）
    bool flush();

private:
    std::string filename_;      // 保存的文件名
    std::ofstream file_;        // 输出文件流（ofstream = output file stream）
    bool isOpen_;               // 标记文件是否已打开
    int sampleCount_;           // 已记录的样本数量

    // 写入 CSV 表头（第一行，说明每列是什么数据）
    void writeHeader();
};

#endif
