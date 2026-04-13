"""
SO101 Leader -> SO101 Follower 映射。

职责：
1. 维护电机/关节的命名与顺序。
2. 定义真实电机的量程（校准归一化后）与仿真关节的角度限制。
3. 提供从 leader 读数到仿真关节指令的转换函数。
4. 预留滤波/裁剪接口，后续可在这里插入低通滤波、限幅等逻辑。
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Tuple

import numpy as np

# 关节顺序，保持硬件-仿真一致
JOINT_ORDER: List[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# 电机量程（归一化后），拷贝自 leisaac/lerobot 规范
SO101_FOLLOWER_MOTOR_LIMITS: Dict[str, Tuple[float, float]] = {
    "shoulder_pan": (-100.0, 100.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 100.0),
    "wrist_flex": (-100.0, 100.0),
    "wrist_roll": (-100.0, 100.0),
    "gripper": (0.0, 100.0),
}

# 仿真关节限制（度），拷贝自 leisaac/lerobot 规范
SO101_FOLLOWER_USD_JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 90.0),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-160.0, 160.0),
    "gripper": (-10.0, 100.0),
}


def leader_to_sim_radians(joint_state: Dict[str, float]) -> np.ndarray:
    """
    将 leader 读到的归一化电机位置映射到仿真关节角（弧度）。
    joint_state：键为 JOINT_ORDER 中的名字，值为归一化电机读数（-100~100 或 0~100）。
    返回：按 JOINT_ORDER 排列的弧度数组。
    """
    processed = np.zeros(len(JOINT_ORDER), dtype=np.float32)
    for idx, joint_name in enumerate(JOINT_ORDER):
        motor_min, motor_max = SO101_FOLLOWER_MOTOR_LIMITS[joint_name]
        joint_min, joint_max = SO101_FOLLOWER_USD_JOINT_LIMITS[joint_name]
        motor_range = motor_max - motor_min
        joint_range = joint_max - joint_min
        motor_value = joint_state[joint_name] - motor_min
        joint_deg = motor_value / motor_range * joint_range + joint_min
        processed[idx] = np.deg2rad(joint_deg)
    return processed


def clamp_to_limits(rad_targets: Iterable[float]) -> np.ndarray:
    """按关节限制裁剪弧度指令。"""
    rad_targets = np.asarray(rad_targets, dtype=np.float32)
    clamped = np.zeros_like(rad_targets)
    for idx, joint_name in enumerate(JOINT_ORDER):
        deg_min, deg_max = SO101_FOLLOWER_USD_JOINT_LIMITS[joint_name]
        rad_min, rad_max = np.deg2rad(deg_min), np.deg2rad(deg_max)
        clamped[idx] = np.clip(rad_targets[idx], rad_min, rad_max)
    return clamped


class LowPassFilter:
    """简易一阶低通滤波器，预留供后续平滑使用。"""

    def __init__(self, alpha: float):
        self.alpha = alpha
        self._state: np.ndarray | None = None

    def reset(self):
        self._state = None

    def __call__(self, value: np.ndarray) -> np.ndarray:
        if self._state is None:
            self._state = value.copy()
        else:
            self._state = self.alpha * value + (1.0 - self.alpha) * self._state
        return self._state

