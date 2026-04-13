"""
SO101 Leader 设备驱动（封装 Feetech 总线）。

职责：
- 连接/断开/校准真实 SO101 Leader。
- 读取各关节的当前位置（已按校准归一化）。
- 暴露电机量程供映射层使用。
"""

from __future__ import annotations

import json
import os
from typing import Dict, Tuple

import numpy as np

from hardware.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from hardware.common.motors.motors_bus import Motor, MotorCalibration, MotorNormMode
from hardware.common.motors.feetech.feetech import FeetechMotorsBus, OperatingMode
from mapping.so101_mapping import JOINT_ORDER, SO101_FOLLOWER_MOTOR_LIMITS


class SO101Leader:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        recalibrate: bool = False,
        calibration_file_name: str = "so101_leader_calib.json",
        connect_on_init: bool = True,
    ):
        self.port = port
        self.calibration_path = os.path.join(os.path.dirname(__file__), ".cache", calibration_file_name)

        # 初始化校准文件
        if not os.path.exists(self.calibration_path) or recalibrate:
            self.calibrate()
        calibration = self._load_calibration()

        # 初始化总线
        self._bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=calibration,
        )
        self._motor_limits = SO101_FOLLOWER_MOTOR_LIMITS
        self._saved_torque_limits: Dict[str, int] = {}
        if connect_on_init:
            self.connect()

    def __str__(self) -> str:
        return "SO101-Leader 设备，用于实时遥操作 SO101-Follower"

    # ------------ 公共接口 ------------
    def connect(self):
        if self.is_connected:
            raise DeviceAlreadyConnectedError("SO101-Leader 已连接。")
        self._bus.connect()
        self.configure()
        print("SO101-Leader connected.")

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError("SO101-Leader 未连接。")
        self._bus.disconnect()
        print("SO101-Leader disconnected.")

    @property
    def is_connected(self) -> bool:
        return self._bus.is_connected

    @property
    def motor_limits(self) -> Dict[str, Tuple[float, float]]:
        return self._motor_limits

    def get_joint_state(self) -> Dict[str, float]:
        """
        返回归一化后的电机位置（键为 JOINT_ORDER）。
        """
        state = self._bus.sync_read("Present_Position")
        return {name: state[name] for name in JOINT_ORDER}

    def enable_torque(self, motors: list[str] | None = None):
        """
        开启指定电机扭矩（默认全部）。
        注意：力反馈需要扭矩开启，否则写 Goal_Position 不会产生“阻力/推回”效果。
        """
        self._bus.enable_torque(motors)

    def disable_torque(self, motors: list[str] | None = None):
        """关闭指定电机扭矩（默认全部）。"""
        self._bus.disable_torque(motors)

    def apply_position_offset(self, offsets: Dict[str, float]):
        """
        给每个关节施加小幅偏置（单位同归一化输入），用于模拟力反馈。
        偏置会被电机量程裁剪。
        """
        current_state = self.get_joint_state()
        cmd = {}
        for name, delta in offsets.items():
            min_v, max_v = self._motor_limits[name]
            current = current_state[name]
            target = np.clip(current + delta, min_v, max_v)
            cmd[name] = target
        self._bus.sync_write("Goal_Position", cmd)

    def hold_current_position(self, motors: list[str] | None = None):
        """
        将指定电机的 Goal_Position 设为当前 Present_Position。

        典型用途：在 Torque_Enable 由 0->1 前调用，避免电机瞬间跳到旧的 Goal_Position。
        """
        current_state = self.get_joint_state()
        if motors is None:
            cmd = {name: current_state[name] for name in JOINT_ORDER}
        else:
            cmd = {name: current_state[name] for name in motors}
        self._bus.sync_write("Goal_Position", cmd)

    def set_torque_limit_fraction(self, motor: str, fraction: float):
        """
        将指定电机的 Torque_Limit 临时按比例缩放（0~1），用于让力反馈更“软”。

        - 首次调用会缓存当前 Torque_Limit，便于后续 restore。
        - fraction<=0 会把 Torque_Limit 置 0（不建议在使用中这样做）。
        """
        if motor not in self._saved_torque_limits:
            self._saved_torque_limits[motor] = int(self._bus.read("Torque_Limit", motor, normalize=False))
        base = self._saved_torque_limits[motor]
        new_limit = int(max(0.0, float(fraction)) * float(base))
        self._bus.write("Torque_Limit", motor, new_limit, normalize=False)

    def restore_torque_limit(self, motor: str):
        """恢复此前缓存的 Torque_Limit（若存在）。"""
        if motor not in self._saved_torque_limits:
            return
        self._bus.write("Torque_Limit", motor, self._saved_torque_limits[motor], normalize=False)
        del self._saved_torque_limits[motor]

    # ------------ 内部方法 ------------
    def configure(self) -> None:
        self._bus.disable_torque()
        self._bus.configure_motors()
        for motor in self._bus.motors:
            self._bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def calibrate(self):
        # 独立的 bus，用于校准过程
        bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
        )
        bus.connect()

        print("\n开始 SO101-Leader 校准")
        bus.disable_torque()
        for motor in bus.motors:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input("把 SO101-Leader 移到中位后按 ENTER...")
        homing_offset = bus.set_half_turn_homings()
        print("依次移动各关节到全行程，按 ENTER 结束记录。")
        range_mins, range_maxes = bus.record_ranges_of_motion()

        calibration = {}
        for motor, m in bus.motors.items():
            calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offset[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
        bus.write_calibration(calibration)
        self._save_calibration(calibration)
        print(f"校准数据已保存到 {self.calibration_path}")

        bus.disconnect()

    def _load_calibration(self) -> Dict[str, MotorCalibration]:
        with open(self.calibration_path) as f:
            json_data = json.load(f)
        calibration: Dict[str, MotorCalibration] = {}
        for motor_name, motor_data in json_data.items():
            calibration[motor_name] = MotorCalibration(
                id=int(motor_data["id"]),
                drive_mode=int(motor_data["drive_mode"]),
                homing_offset=int(motor_data["homing_offset"]),
                range_min=int(motor_data["range_min"]),
                range_max=int(motor_data["range_max"]),
            )
        return calibration

    def _save_calibration(self, calibration: Dict[str, MotorCalibration]):
        payload = {
            k: {
                "id": v.id,
                "drive_mode": v.drive_mode,
                "homing_offset": v.homing_offset,
                "range_min": v.range_min,
                "range_max": v.range_max,
            }
            for k, v in calibration.items()
        }
        os.makedirs(os.path.dirname(self.calibration_path), exist_ok=True)
        with open(self.calibration_path, "w") as f:
            json.dump(payload, f, indent=4)
