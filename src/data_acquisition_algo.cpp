#include "data_acquisition_algo.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace jaka_algo {

DataAcquisitionAlgorithm::DataAcquisitionAlgorithm(const AcquisitionConfig& cfg)
    : config_(cfg), current_target_idx_(0), rng_(std::random_device{}()) {}

void DataAcquisitionAlgorithm::set_config(const AcquisitionConfig& cfg) {
    config_ = cfg;
}

AcquisitionConfig DataAcquisitionAlgorithm::get_config() const {
    return config_;
}

std::vector<std::array<double, 6>> DataAcquisitionAlgorithm::generate_random_points() {
    targets_.clear();
    targets_.reserve(config_.point_count);

    std::uniform_real_distribution<double> dist01(0.0, 1.0);

    for (int i = 0; i < config_.point_count; ++i) {
        std::array<double, 6> point;
        for (int j = 0; j < 6; ++j) {
            double t = dist01(rng_);
            point[j] = config_.lower_limits[j] + t * (config_.upper_limits[j] - config_.lower_limits[j]);
        }
        targets_.push_back(point);
    }
    current_target_idx_ = 0;
    return targets_;
}

std::array<double, 6> DataAcquisitionAlgorithm::get_current_target() const {
    if (current_target_idx_ < targets_.size()) {
        return targets_[current_target_idx_];
    }
    return {0, 0, 0, 0, 0, 0};
}

bool DataAcquisitionAlgorithm::has_next_target() const {
    return current_target_idx_ < targets_.size();
}

void DataAcquisitionAlgorithm::advance_target() {
    if (current_target_idx_ < targets_.size()) {
        ++current_target_idx_;
    }
    recent_samples_.clear();  // 切换点位后清空滑动窗口
}

void DataAcquisitionAlgorithm::feed_sensor_data(const RobotSample& sample) {
    recent_samples_.push_back(sample);
    // 保留最近 stable_frames * 2 帧，防止内存无限增长
    const size_t max_window = static_cast<size_t>(config_.stable_frames) * 2;
    if (recent_samples_.size() > max_window) {
        recent_samples_.erase(recent_samples_.begin());
    }

    // 若满足静止条件，记录该样本
    if (is_stationary()) {
        collected_samples_.push_back(sample);
    }
}

bool DataAcquisitionAlgorithm::is_stationary() const {
    if (recent_samples_.size() < static_cast<size_t>(config_.stable_frames)) {
        return false;
    }

    // 检查最近 stable_frames 帧是否全部满足阈值
    size_t start = recent_samples_.size() - config_.stable_frames;
    for (size_t i = start; i < recent_samples_.size(); ++i) {
        if (!check_all_axes_near_zero(recent_samples_[i].joint_vel, config_.vel_threshold)) {
            return false;
        }
        if (!check_all_axes_near_zero(recent_samples_[i].joint_acc, config_.acc_threshold)) {
            return false;
        }
    }
    return true;
}

bool DataAcquisitionAlgorithm::check_all_axes_near_zero(const std::array<double, 6>& values,
                                                         double threshold) const {
    for (double v : values) {
        if (std::abs(v) > threshold) {
            return false;
        }
    }
    return true;
}

size_t DataAcquisitionAlgorithm::get_collected_count() const {
    return collected_samples_.size();
}

void DataAcquisitionAlgorithm::export_csv(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::out);
    if (!ofs.is_open()) {
        throw std::runtime_error("Failed to open CSV file: " + filename);
    }

    // CSV 头
    ofs << "timestamp,j1,j2,j3,j4,j5,j6,"
        << "tau1,tau2,tau3,tau4,tau5,tau6,"
        << "Fx,Fy,Fz,Mx,My,Mz\n";

    ofs << std::fixed << std::setprecision(6);
    for (const auto& s : collected_samples_) {
        ofs << s.timestamp << ",";
        for (int i = 0; i < 6; ++i) ofs << s.joint_angles[i] << (i < 5 ? "," : ",");
        for (int i = 0; i < 6; ++i) ofs << s.joint_torques[i] << (i < 5 ? "," : ",");
        for (int i = 0; i < 6; ++i) ofs << s.force_torque[i] << (i < 5 ? "," : "\n");
    }
    ofs.close();
}

void DataAcquisitionAlgorithm::reset() {
    current_target_idx_ = 0;
    collected_samples_.clear();
    recent_samples_.clear();
    targets_.clear();
}

} // namespace jaka_algo
