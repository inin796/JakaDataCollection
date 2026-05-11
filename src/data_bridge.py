# data_bridge.py
"""
数据桥接层（核心文件）：
专门实现 Python 调用接口读取到的系统数据赋值为 C++ 控制算法需要用到的变量。
本文件不包含任何 JAKA API 直接调用，仅负责数据类型转换与赋值。
"""

import time
import jaka_algo  # pybind11 编译生成的 C++ 算法模块


class DataBridge:
    """
    将 Python 从 JAKA SDK 读取到的原始数据，逐字段赋值给 C++ 算法结构体。
    """

    def __init__(self, cpp_algo: jaka_algo.DataAcquisitionAlgorithm):
        self.algo = cpp_algo

    # ------------------------------------------------------------------
    # 核心桥接方法：Python 数据 → C++ 变量赋值
    # ------------------------------------------------------------------
    def assign_robot_state_to_cpp(self,
                                   timestamp: float,
                                   joint_pos: list,
                                   joint_torque: list,
                                   force_torque: list,
                                   joint_vel: list,
                                   joint_acc: list) -> None:
        """
        将 Python 调用接口读取到的系统数据赋值为 C++ 控制算法需要用到的变量。

        参数:
            timestamp:    时间戳 [s]
            joint_pos:    关节角 [j1..j6] rad
            joint_torque: 关节扭矩 [tau1..tau6] Nm
            force_torque: 六维力/力矩 [Fx,Fy,Fz,Mx,My,Mz]
            joint_vel:    关节角速度 [v1..v6] rad/s
            joint_acc:    关节加速度 [a1..a6] rad/s^2
        """
        # 创建 C++ RobotSample 对象
        sample = jaka_algo.RobotSample()

        # 逐个字段赋值（显式桥接）
        sample.timestamp = float(timestamp)

        # 确保长度为 6，不足补 0.0
        sample.joint_angles = self._to_array6(joint_pos)
        sample.joint_torques = self._to_array6(joint_torque)
        sample.force_torque = self._to_array6(force_torque)
        sample.joint_vel = self._to_array6(joint_vel)
        sample.joint_acc = self._to_array6(joint_acc)

        # 赋值给 C++ 算法内部变量
        self.algo.feed_sensor_data(sample)

    def check_stationary(self) -> bool:
        """查询 C++ 算法判断当前是否静止。"""
        return self.algo.is_stationary()

    def get_collected_count(self) -> int:
        """获取 C++ 算法已采集的有效样本数。"""
        return self.algo.get_collected_count()

    def trigger_next_point(self) -> None:
        """通知 C++ 算法进入下一个点位。"""
        self.algo.advance_target()

    def export_csv(self, filename: str) -> None:
        """触发 C++ 算法导出 CSV。"""
        self.algo.export_csv(filename)

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------
    @staticmethod
    def _to_array6(data) -> list:
        """将输入数据转换为 6 元素列表，不足补 0.0。"""
        if data is None:
            return [0.0] * 6
        arr = list(data)[:6]
        while len(arr) < 6:
            arr.append(0.0)
        return arr
