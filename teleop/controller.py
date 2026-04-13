from __future__ import annotations

import time
from typing import Optional

import numpy as np

from hardware.leader.so101_leader import SO101Leader
from mapping.force_mapping import ForceLowPass, wrench_to_leader_offset
from mapping.so101_mapping import LowPassFilter, leader_to_sim_radians
from sim.base.environment import SimInterface


class RateLimiter:
    """简单的频率控制器。"""

    def __init__(self, hz: float):
        if hz <= 0:
            raise ValueError(f"hz must be > 0, got {hz}")
        self.period = 1.0 / hz
        self._next = time.monotonic()

    def sleep(self):
        now = time.monotonic()
        if now < self._next:
            time.sleep(self._next - now)
            self._next = self._next + self.period
            return
        # 已经超时：重置节拍，避免长期追帧
        self._next = now + self.period


def run_teleop(
    leader: SO101Leader,
    sim_env: SimInterface,
    hz: float = 60.0,
    lowpass_alpha: Optional[float] = None,
    force_feedback: bool = False,
    ff_scale: float = 0.01,
    ff_max_offset: float = 5.0,
    ff_lowpass_alpha: Optional[float] = 0.2,
    ff_contact_thresh: float = 0.2,
    # 默认只用活动爪指：gripper 舵机只驱动它；固定爪指接触不应触发“关夹爪阻力”
    ff_bodies: tuple[str, ...] = ("moving_jaw_so101_v1",),
    ff_direction: str = "open",
    ff_deadband: float = 0.02,
    ff_torque_fraction: float = 0.3,
    ff_debug: bool = False,
):
    """
    循环：读取 leader → 映射 → 仿真。
    lowpass_alpha：若提供，则使用一阶低通平滑指令（0~1，越小越平滑）。
    force_feedback: 开启后读取仿真末端受力，向 leader 施加小幅“推回”。
    """
    rate = RateLimiter(hz)
    filt = LowPassFilter(lowpass_alpha) if lowpass_alpha is not None else None
    ff_filt = ForceLowPass(ff_lowpass_alpha) if ff_lowpass_alpha is not None else None
    ff_torque_on = False
    prev_gripper: float | None = None
    last_debug_print = 0.0
    last_warn_print = 0.0

    while True:
        # 读硬件
        joint_state = leader.get_joint_state()

        gripper_pos = float(joint_state["gripper"])
        gripper_delta = 0.0 if prev_gripper is None else (gripper_pos - prev_gripper)
        prev_gripper = gripper_pos

        # 转换到仿真关节弧度
        targets = leader_to_sim_radians(joint_state)
        if filt:
            targets = filt(targets)
        # 送入仿真
        sim_env.apply_joint_positions(targets)

        # 力反馈（可选）：仅在接触力超过阈值时触发
        if force_feedback:
            wrench_sum_raw = np.zeros(6, dtype=np.float32)
            per_body_force_norm: dict[str, float] = {}
            for body in ff_bodies:
                w = np.asarray(sim_env.get_end_effector_wrench(body_name=body), dtype=np.float32)
                if w.shape[0] >= 6:
                    wrench_sum_raw += w[:6]
                elif w.shape[0] >= 3:
                    wrench_sum_raw[:3] += w[:3]
                per_body_force_norm[body] = float(np.linalg.norm(w[:3]))

            raw_force_norm = float(np.linalg.norm(wrench_sum_raw[:3]))

            # 没有接触时立刻清零并重置滤波器：避免接触结束后仍残留“尾巴”阻力。
            if raw_force_norm <= float(ff_contact_thresh):
                if ff_filt:
                    ff_filt.reset()
                wrench_sum = np.zeros(6, dtype=np.float32)
            else:
                wrench_sum = wrench_sum_raw
                if ff_filt:
                    wrench_sum = ff_filt(wrench_sum)

            force_norm = float(np.linalg.norm(wrench_sum[:3]))
            offset = wrench_to_leader_offset(
                wrench_sum,
                scale=ff_scale,
                max_offset=ff_max_offset,
                contact_thresh=ff_contact_thresh,
            )

            # 只在“用户正朝被抵抗的方向运动”且接触力超过阈值时才打开扭矩并施加反馈；
            # 否则关闭扭矩，避免 leader 夹爪在持续接触时被“锁死”。
            oppose_motion = False
            if ff_direction == "open":
                # open=推向张开：用于“抵抗关夹爪”，所以只在用户正在关闭时触发
                oppose_motion = gripper_delta < -ff_deadband
            elif ff_direction == "close":
                # close=推向闭合：用于“抵抗开夹爪”，所以只在用户正在张开时触发
                oppose_motion = gripper_delta > ff_deadband

            if oppose_motion and offset != 0.0:
                if not ff_torque_on:
                    # 防止扭矩开启瞬间跳到旧 Goal_Position：先把目标设为当前位姿再开扭矩。
                    try:
                        leader.hold_current_position(["gripper"])
                    except Exception:
                        pass
                    try:
                        leader.set_torque_limit_fraction("gripper", ff_torque_fraction)
                    except Exception:
                        pass
                    try:
                        leader.enable_torque(["gripper"])
                        ff_torque_on = True
                    except Exception as exc:
                        now = time.monotonic()
                        if now - last_warn_print > 2.0:
                            print(f"[ff] enable_torque(gripper) failed: {exc}")
                            last_warn_print = now
                        try:
                            leader.restore_torque_limit("gripper")
                        except Exception:
                            pass
                        ff_torque_on = False
                if ff_torque_on:
                    signed = -abs(offset) if ff_direction == "open" else +abs(offset)
                    try:
                        leader.apply_position_offset({"gripper": signed})
                    except Exception as exc:
                        now = time.monotonic()
                        if now - last_warn_print > 2.0:
                            print(f"[ff] apply_position_offset(gripper) failed: {exc}")
                            last_warn_print = now
            else:
                if ff_torque_on:
                    try:
                        leader.restore_torque_limit("gripper")
                    except Exception:
                        pass
                    try:
                        leader.disable_torque(["gripper"])
                    except Exception:
                        pass
                    ff_torque_on = False

            if ff_debug:
                now = time.monotonic()
                if now - last_debug_print > 0.5:
                    bodies_str = " ".join([f"{k}={v:.3f}" for k, v in per_body_force_norm.items()])
                    print(
                        f"[ff] raw|F|={raw_force_norm:.3f} |F|={force_norm:.3f} offset={offset:.4f} delta={gripper_delta:.3f} "
                        f"oppose={oppose_motion} torque={ff_torque_on} bodies[{bodies_str}]"
                    )
                    last_debug_print = now
        rate.sleep()
