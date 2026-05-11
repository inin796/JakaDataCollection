#ifndef DATA_ACQUISITION_ALGO_H
#define DATA_ACQUISITION_ALGO_H

#include <array>
#include <vector>
#include <string>
#include <random>
#include <fstream>

namespace jaka_algo {

struct RobotSample {
    double timestamp;                     // 时间戳 [s]
    std::array<double, 6> joint_angles;   // 关节角 [rad]
    std::array<double, 6> joint_torques;  // 关节扭矩 [Nm]
    std::array<double, 6> force_torque;   // 六维力/力矩 [Fx,Fy,Fz,Mx,My,Mz]
    std::array<double, 6> joint_vel;      // 关节角速度 [rad/s]
    std::array<double, 6> joint_acc;      // 关节加速度 [rad/s^2]
};

struct AcquisitionConfig {
    std::array<double, 6> lower_limits;   // 关节下限 [rad]
    std::array<double, 6> upper_limits;   // 关节上限 [rad]
    int point_count = 50;                 // 随机点位数量
    double vel_threshold = 0.001;         // 静止速度阈值 [rad/s]
    double acc_threshold = 0.001;         // 静止加速度阈值 [rad/s^2]
    int stable_frames = 10;               // 连续满足静止条件的帧数
};

class DataAcquisitionAlgorithm {
public:
    explicit DataAcquisitionAlgorithm(const AcquisitionConfig& cfg = AcquisitionConfig());

    // 设置/获取配置
    void set_config(const AcquisitionConfig& cfg);
    AcquisitionConfig get_config() const;

    // 在限定范围内生成随机点位（关节空间）
    std::vector<std::array<double, 6>> generate_random_points();

    // 获取当前目标点位（供 Python 查询）
    std::array<double, 6> get_current_target() const;
    bool has_next_target() const;
    void advance_target();                // 切换到下一个点位

    // 接收来自 Python 桥接层的传感器数据
    void feed_sensor_data(const RobotSample& sample);

    // 判断当前是否满足静止条件（vel≈0, acc≈0）
    bool is_stationary() const;

    // 获取已采集的有效样本数
    size_t get_collected_count() const;

    // 导出 CSV（包含关节角、关节扭矩、六维力、时间）
    void export_csv(const std::string& filename) const;

    // 重置算法状态
    void reset();

private:
    AcquisitionConfig config_;
    std::vector<std::array<double, 6>> targets_;
    size_t current_target_idx_;
    std::vector<RobotSample> collected_samples_;

    // 用于静止判断的滑动窗口
    std::vector<RobotSample> recent_samples_;
    mutable std::mt19937 rng_;

    bool check_all_axes_near_zero(const std::array<double, 6>& values, double threshold) const;
};

} // namespace jaka_algo

#endif
