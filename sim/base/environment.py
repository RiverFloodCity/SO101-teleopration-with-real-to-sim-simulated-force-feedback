"""仿真统一接口，便于切换后端（Mujoco/Isaac/Gazebo 等）。"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Iterable, Sequence


class SimInterface(ABC):
    """仿真后端基类。"""

    @abstractmethod
    def connect(self): ...

    @abstractmethod
    def reset(self): ...

    @abstractmethod
    def apply_joint_positions(self, joint_positions_rad: Iterable[float]): ...

    @abstractmethod
    def close(self): ...

    def get_end_effector_wrench(self, body_name: str = "gripper") -> Sequence[float]:
        """
        可选：返回末端 6D 力/矩（Fx,Fy,Fz,Mx,My,Mz）。
        默认返回 0（表示该后端不支持/未启用接触力反馈）。
        """
        _ = body_name
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
