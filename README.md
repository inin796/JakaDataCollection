# jaka_data_collection 项目大纲

## 项目根目录
- `CMakeLists.txt` — 顶层 CMake 构建配置文件

## C++ 纯算法层（零 API 调用）
- `data_acquisition_algo.h`  
- `data_acquisition_algo.cpp`  

## pybind11 绑定层（零 API 调用）
- `algo_binding.cpp`  
  编译命令（需安装 pybind11）：
  c++ -O3 -Wall -shared -std=c++14 -fPIC \
    $(python3 -m pybind11 --includes) \
    algo_binding.cpp data_acquisition_algo.cpp \
    -o jaka_algo$(python3-config --extension-suffix)

## Python 桥接层（专门的数据赋值文件）
- `data_bridge.py` 

## Python 桥接层（专门的数据赋值文件）
- `Python 硬件接口层（唯一 API 调用）`

## 主控流程
- `main.py`
