# jaka_data_collection 项目大纲
## 文件架构与约束说明
| 文件                             | 语言     | 职责                                    | 是否含 JAKA API |
| ------------------------------ | ------ | ------------------------------------- | ------------ |
| `robot_interface.py`           | Python | **唯一**封装 `jkrc` 登录、运动、状态读取、力传感器配置     | ✅ 唯一允许       |
| `data_bridge.py`               | Python | **数据桥接**：将 Python 读取的系统数据赋值给 C++ 算法变量 | ❌ 仅做数据转换     |
| `data_acquisition_algo.h/.cpp` | C++    | **纯算法**：随机点位生成、静止判断、数据缓存、CSV 导出       | ❌ 零 API 调用   |
| `algo_binding.cpp`             | C++    | `pybind11` 绑定层                        | ❌ 零 API 调用   |
| `main.py`                      | Python | 主控流程：连接 → 生成点位 → 运动 → 采集 → 导出 CSV     | ❌ 仅调用前两层     |


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
