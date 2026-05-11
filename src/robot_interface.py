# robot_interface.py
"""
唯一允许直接调用 JAKA Python SDK (jkrc) 的文件。
封装连接、运动、状态读取、力传感器配置等全部硬件交互。
"""

import time
import jkrc


class JakaRobotInterface:
    ABS = 0
    INCR = 1

    def __init__(self, ip: str = "192.168.2.194",
                 user: str = "jaka_sdk",
                 password: str = "password"):
        self.ip = ip
        self.user = user
        self.password = password
        self.robot = jkrc.RC(ip)
        self._connected = False

    def connect(self):
        """登录、上电、使能 [^42^]"""
        ret = self.robot.login(1, self.user, self.password)
        if ret[0] != 0:
            raise ConnectionError(f"登录失败，错误码: {ret[0]}")
        self.robot.power_on()
        self.robot.enable_robot()
        self._connected = True
        return True

    def disconnect(self):
        """安全断开"""
        if self._connected:
            self.robot.disable_robot()
            self.robot.power_off()
            self.robot.logout()
            self._connected = False

    def init_force_sensor(self, sensor_brand: int = 10):
        """
        初始化末端力传感器。
        sensor_brand: 10 表示法兰内置传感器，1-6 为外接型号 [^8^]
        """
        # 开启力传感器
        ret = self.robot.set_torque_sensor_mode(1)
        if ret[0] != 0:
            print(f"[WARN] 开启力传感器失败: {ret[0]}")
        # 零点校准 + 恒力柔顺控制类型
        ret = self.robot.set_compliant_type(1, 0)  # 先补偿 [^8^]
        time.sleep(1.0)
        ret = self.robot.set_compliant_type(0, 1)  # 再开启恒力控制
        if ret[0] != 0:
            print(f"[WARN] 设置力控类型失败: {ret[0]}")

    def joint_move(self, joint_pos: list, speed: float = 0.5, is_block: bool = True):
        """关节空间点到点运动 [^42^]"""
        return self.robot.joint_move(joint_pos, self.ABS, is_block, speed)

    def get_robot_status_dict(self) -> dict:
        """
        获取机器人完整监控数据并解析为字典。
        返回字段基于 JAKA RobotStatus 结构体推断 [^17^][^24^]，
        实际索引需根据 SDK 版本通过调试确认。
        """
        ret = self.robot.get_robot_status()
        if ret[0] != 0:
            return {}

        raw = ret[1]
        # ------------------------------------------------------------
        # 注意：以下索引为基于 C++ RobotStatus 结构的推断。
        # 实际使用时请先用 print(raw) 确认字段顺序。
        # 若 get_robot_status 有 bug 或字段不足，可降级使用
        # get_joint_position() + TCP/IP 协议获取力传感器数据 [^17^][^36^]
        # ------------------------------------------------------------
        try:
            # 假设 raw[0:6]  为 jointMonitorData[0..5].instVel
            # 假设 raw[6:12] 为 jointMonitorData[0..5].instTorq
            # 假设 raw[12:18] 为 actTorque[0..5]（六维力/力矩）
            # 假设 raw[18:24] 为 TCP 位姿（用于校验）
            # 以上仅为示例映射，请根据实际 SDK 输出调整
            joint_vel = list(raw[0:6]) if len(raw) >= 6 else [0.0] * 6
            joint_torque = list(raw[6:12]) if len(raw) >= 12 else [0.0] * 6
            force_torque = list(raw[12:18]) if len(raw) >= 18 else [0.0] * 6
        except Exception as e:
            print(f"[WARN] 解析 get_robot_status 失败: {e}")
            joint_vel = [0.0] * 6
            joint_torque = [0.0] * 6
            force_torque = [0.0] * 6

        # 关节角使用独立的可靠接口
        ret_pos = self.robot.get_joint_position()
        joint_pos = list(ret_pos[1]) if ret_pos[0] == 0 else [0.0] * 6

        # 加速度在 JAKA 标准 SDK 中不直接提供，此处以 0 占位
        # 若需实际加速度，可通过速度差分计算或启用 EDG 模式
        joint_acc = [0.0] * 6

        return {
            'timestamp': time.time(),
            'joint_pos': joint_pos,
            'joint_vel': joint_vel,
            'joint_acc': joint_acc,
            'joint_torque': joint_torque,
            'force_torque': force_torque,
        }

    def is_motion_done(self, tol: float = 0.001) -> bool:
        """简单判断机器人是否已停止运动（速度近似为0）"""
        status = self.get_robot_status_dict()
        if not status:
            return False
        vel = status.get('joint_vel', [0.0] * 6)
        return all(abs(v) < tol for v in vel)
