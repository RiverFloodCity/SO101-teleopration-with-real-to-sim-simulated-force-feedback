"""
力反馈映射：
- 从仿真末端 6D 力/矩 -> 标量/偏置。
- 预留低通滤波、限幅。
"""

from __future__ import annotations

import numpy as np


class ForceLowPass:
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


def wrench_to_leader_offset(
    wrench: np.ndarray,
    scale: float,
    max_offset: float,
    contact_thresh: float = 0.0,
) -> float:
    """
    将 6D 力/矩映射为一个标量偏置（单位：leader 归一化位置）。

    - 输入 wrench 单位：MuJoCo 的外力/力矩（近似 N / N·m）
    - 输出 offset 单位：leader 的归一化位置（RANGE_0_100 / RANGE_M100_100）

    映射策略：
    - 取力的模长 ||F||（仅使用 wrench[:3]）
    - 扣掉触发阈值 contact_thresh（<=0 表示不扣）
    - 乘以比例系数 scale（单位：leader_unit / N）
    - 限幅到 [-max_offset, +max_offset]
    """
    force_norm = float(np.linalg.norm(wrench[:3]))  # 仅取力
    effective = max(0.0, force_norm - float(contact_thresh))
    raw = effective * float(scale)
    max_abs = abs(float(max_offset))
    return float(np.clip(raw, -max_abs, max_abs))


# Backward-compatible alias (older name in this repo)
def wrench_to_offset(wrench: np.ndarray, scale: float = 0.01, max_offset_deg: float = 2.0) -> float:
    return wrench_to_leader_offset(wrench, scale=scale, max_offset=max_offset_deg, contact_thresh=0.0)
