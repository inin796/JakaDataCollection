#include "data_recorder.h"
#include <iostream>
#include <iomanip>

DataRecorder::DataRecorder(const std::string& filename)
    : filename_(filename), isOpen_(false), sampleCount_(0) {}

DataRecorder::~DataRecorder() { close(); }

bool DataRecorder::open() {
    file_.open(filename_, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        std::cerr << "[DataRecorder] Failed to open: " << filename_ << std::endl;
        return false;
    }
    isOpen_ = true;
    writeHeader();
    std::cout << "[DataRecorder] Opened: " << filename_ << std::endl;
    return true;
}

bool DataRecorder::close() {
    if (isOpen_) {
        flush();
        file_.close();
        isOpen_ = false;
        std::cout << "[DataRecorder] Closed. Samples: " << sampleCount_ << std::endl;
    }
    return true;
}

void DataRecorder::writeHeader() {
    file_ << "point_index,timestamp,"
          << "j1,j2,j3,j4,j5,j6,"
          << "torque1,torque2,torque3,torque4,torque5,torque6,"
          << "fx,fy,fz,tx,ty,tz\n";
    file_.flush();
}

bool DataRecorder::record(const RobotDataSample& sample) {
    if (!isOpen_) return false;
    file_ << sample.pointIndex << ","
          << std::fixed << std::setprecision(6) << sample.timestamp << ",";

    for (int i = 0; i < 6; ++i) file_ << sample.angles.j[i] << (i < 5 ? "," : "");
    file_ << ",";
    for (int i = 0; i < 6; ++i) file_ << sample.torques.t[i] << (i < 5 ? "," : "");
    file_ << ",";

    file_ << sample.force.fx << "," << sample.force.fy << "," << sample.force.fz << ","
          << sample.force.tx << "," << sample.force.ty << "," << sample.force.tz << "\n";

    ++sampleCount_;
    if (sampleCount_ % 10 == 0) file_.flush();
    return true;
}

bool DataRecorder::flush() {
    if (isOpen_) file_.flush();
    return true;
}
