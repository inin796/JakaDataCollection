# jaka_data_collection 项目大纲

## 项目根目录
- `CMakeLists.txt` — 顶层 CMake 构建配置文件

## include/ — 公共头文件层
- `types.h`  
  纯数据结构定义，跨所有模块共享，无外部依赖
- `robot_interface.h`  
  抽象机器人接口层，定义统一的机器人操作规范
- `jaka_robot_interface.h`  
  JAKA API 实现层头文件，继承自抽象接口，封装 JAKA 专用功能
- `control_engine.h`  
  控制算法层接口，不包含任何 SDK 头文件，保持平台无关
- `data_recorder.h`  
  数据记录层接口，负责 CSV 文件的生成与写入

## src/ — 源文件实现层
- `main.cpp`  
  程序入口，负责初始化、组装各模块并运行主循环
- `jaka_robot_interface.cpp`  
  **唯一允许包含 JAKA SDK 头文件的源文件**，实现 JAKA 机器人具体交互
- `control_engine.cpp`  
  纯控制逻辑实现，不依赖特定机器人 SDK
- `data_recorder.cpp`  
  数据记录器实现，将运行时数据写入 CSV 文件

## c&c++/ — 第三方 JAKA SDK（用户自行放置）
- `inc_of_c++/` — SDK 头文件目录
  - `JAKAZuRobot.h`
  - `jktypes.h`
  - `jkerr.h`
- `x86_64-linux-gnu/shared/` — 动态链接库目录
  - `libjakaAPI.so`
