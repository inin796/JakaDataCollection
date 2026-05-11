# main.py
"""
主控程序：
1. Python 通过 robot_interface 调用 JAKA API 连接机器人、配置力传感器；
2. C++ 算法生成随机点位；
3. Python 控制机器人依次运动到各点位并等待静止；
4. Python 读取系统状态，通过 data_bridge 赋值给 C++ 变量；
5. C++ 算法判断静止后记录数据，最终导出 CSV。
"""

import time
import math
from robot_interface import JakaRobotInterface
from data_bridge import DataBridge
import jaka_algo


def main():
    # ==================== 配置参数 ====================
    ROBOT_IP = "192.168.2.194"
    SPEED_MOVE = 0.3          # 关节运动速度 rad/s
    STABLE_WAIT = 2.0         # 到达点位后额外等待时间 s
    CSV_PATH = "./jaka_acquisition_data.csv"

    # C++ 算法配置：限定运动范围 + 随机点位数量
    cfg = jaka_algo.AcquisitionConfig()
    # 示例：JAKA Zu 系列典型关节限位（弧度），请根据实际型号修改
    cfg.lower_limits = [
        math.radians(-270), math.radians(-120), math.radians(-170),
        math.radians(-270), math.radians(-120), math.radians(-270)
    ]
    cfg.upper_limits = [
        math.radians(270), math.radians(120), math.radians(170),
        math.radians(270), math.radians(120), math.radians(270)
    ]
    cfg.point_count = 30       # 自行确定的随机点位数量
    cfg.vel_threshold = 0.001  # 静止判断阈值 rad/s
    cfg.acc_threshold = 0.001  # 静止判断阈值 rad/s^2
    cfg.stable_frames = 10     # 连续 10 帧满足阈值才判定为静止

    # ==================== 初始化 C++ 算法 ====================
    algo = jaka_algo.DataAcquisitionAlgorithm(cfg)
    targets = algo.generate_random_points()
    print(f"[C++ 算法] 已生成 {len(targets)} 个随机点位")

    # ==================== 初始化桥接层 ====================
    bridge = DataBridge(algo)

    # ==================== Python API 连接机器人 ====================
    robot = JakaRobotInterface(ROBOT_IP)
    robot.connect()
    print("[Python API] 机器人已连接、上电、使能")

    # 配置力传感器（若机器人带末端力传感器）
    robot.init_force_sensor(sensor_brand=10)
    print("[Python API] 力传感器已初始化")

    # ==================== 逐点运动与采集 ====================
    point_idx = 0
    while algo.has_next_target():
        target = algo.get_current_target()
        point_idx += 1
        print(f"\n>>> 点位 {point_idx}/{len(targets)}: "
              f"[{', '.join(f'{math.degrees(v):.1f}°' for v in target)}]")

        # Python 调用 API 运动到目标点位（阻塞直到到达）
        ret = robot.joint_move(target, speed=SPEED_MOVE, is_block=True)
        if ret[0] != 0:
            print(f"[ERROR] 运动失败，错误码: {ret[0]}，跳过该点")
            algo.advance_target()
            continue

        # 等待机器人完全静止
        print("    等待静止...")
        stable_count = 0
        while stable_count < cfg.stable_frames:
            # Python 读取系统数据
            state = robot.get_robot_status_dict()

            # 通过桥接层赋值给 C++ 算法变量
            bridge.assign_robot_state_to_cpp(
                timestamp=state['timestamp'],
                joint_pos=state['joint_pos'],
                joint_torque=state['joint_torque'],
                force_torque=state['force_torque'],
                joint_vel=state['joint_vel'],
                joint_acc=state['joint_acc']
            )

            # 查询 C++ 算法是否判定为静止
            if bridge.check_stationary():
                stable_count += 1
            else:
                stable_count = 0

            time.sleep(0.05)  # 20Hz 采样

        print(f"    已静止，累计采集样本: {bridge.get_collected_count()}")

        # 额外等待确保稳定（可选）
        time.sleep(STABLE_WAIT)

        # 通知 C++ 算法进入下一个点位
        bridge.trigger_next_point()

    # ==================== 导出 CSV ====================
    print(f"\n>>> 采集完成，共记录 {bridge.get_collected_count()} 条数据")
    bridge.export_csv(CSV_PATH)
    print(f">>> CSV 已导出至: {CSV_PATH}")

    # ==================== 断开连接 ====================
    robot.disconnect()
    print(">>> 程序安全结束")


if __name__ == "__main__":
    main()
