#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H

#include "types.h"
#include <string>
#include <fstream>

class DataRecorder {
public:
    explicit DataRecorder(const std::string& filename);
    ~DataRecorder();

    bool open();
    bool close();
    bool record(const RobotDataSample& sample);
    bool flush();

private:
    std::string filename_;
    std::ofstream file_;
    bool isOpen_;
    int sampleCount_;

    void writeHeader();
};

#endif
