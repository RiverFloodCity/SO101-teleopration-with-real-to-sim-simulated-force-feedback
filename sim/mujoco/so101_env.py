from __future__ import annotations

import atexit
import os
import tempfile
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import numpy as np

from mapping.so101_mapping import JOINT_ORDER, clamp_to_limits
from sim.base.environment import SimInterface


class MujocoSO101Env(SimInterface):
    """
    Mujoco 后端：加载 SO101 模型，用 position actuator 动力学驱动关节并步进仿真，
    支持附加物体（地面/方块等）。
    """

    def __init__(
        self,
        model_path: str,
        render: bool = True,
        extras: Optional[List[str]] = None,
        control_dt: Optional[float] = None,
        steps_per_control: Optional[int] = None,
    ):
        self.model_path = model_path
        self.render = render
        self.extras = extras or []
        self.control_dt = control_dt
        self.steps_per_control = steps_per_control
        self.mj = None
        self.model = None
        self.data = None
        self.viewer = None
        self.qpos_idx: Dict[str, int] = {}
        self.actuator_id: Dict[str, int] = {}
        self._resolved_steps_per_control: int = 1
        self._tmp_combined_path: Optional[Path] = None
        self._atexit_registered = False

    def connect(self):
        try:
            import mujoco as mj  # type: ignore
            from mujoco import viewer  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise ImportError("需要安装 mujoco>=3.1.0 才能使用 Mujoco 后端") from exc

        self.mj = mj
        if self.extras:
            self.model = self._load_with_extras(mj)
        else:
            self.model = mj.MjModel.from_xml_path(self.model_path)
        self.data = mj.MjData(self.model)

        if not self._atexit_registered:
            atexit.register(self.close)
            self._atexit_registered = True

        # 解析关节索引
        for name in JOINT_ORDER:
            jnt_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)
            if jnt_id < 0:
                raise ValueError(f"模型中未找到关节: {name}")
            self.qpos_idx[name] = self.model.jnt_qposadr[jnt_id]

        # 解析 actuator 索引（用于 position actuator 驱动）
        for name in JOINT_ORDER:
            act_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, name)
            if act_id < 0:
                raise ValueError(f"模型中未找到 actuator: {name}（需要 position actuator 才能动力学驱动）")
            self.actuator_id[name] = act_id

        timestep = float(self.model.opt.timestep)
        if self.steps_per_control is not None:
            self._resolved_steps_per_control = max(1, int(self.steps_per_control))
        elif self.control_dt is not None and self.control_dt > 0 and timestep > 0:
            # 让仿真时间尽量跟上控制周期：control_dt ~= n * timestep
            self._resolved_steps_per_control = max(1, int(round(self.control_dt / timestep)))
        else:
            self._resolved_steps_per_control = 1

        if self.render:
            try:
                self.viewer = viewer.launch_passive(self.model, self.data)
            except Exception:
                self.viewer = None

    def _load_with_extras(self, mj):
        """合成 base xml + extras（追加到 worldbody），写临时文件再加载，保持相对资产路径。"""
        import xml.etree.ElementTree as ET

        base_tree = ET.parse(self.model_path)
        base_root = base_tree.getroot()
        worldbody = base_root.find("worldbody")
        if worldbody is None:
            raise ValueError("XML 缺少 worldbody")

        def ensure_base_asset():
            asset = base_root.find("asset")
            if asset is not None:
                return asset
            asset = ET.Element("asset")
            wb = base_root.find("worldbody")
            if wb is None:
                base_root.append(asset)
                return asset
            children = list(base_root)
            base_root.insert(children.index(wb), asset)
            return asset

        def merge_asset_from_extra(extra_root):
            extra_asset = extra_root.find("asset")
            if extra_asset is None:
                return
            base_asset = ensure_base_asset()
            for child in list(extra_asset):
                base_asset.append(child)

        def ensure_collision_bits(xml_root):
            # 仅在缺省时补齐，避免覆盖用户自定义碰撞分组
            for geom in xml_root.findall(".//geom"):
                if geom.get("class") == "collision" or geom.get("group") == "3":
                    geom.set("contype", geom.get("contype", "1"))
                    geom.set("conaffinity", geom.get("conaffinity", "1"))

        for extra_path in self.extras:
            extra_tree = ET.parse(extra_path)
            extra_root = extra_tree.getroot()
            merge_asset_from_extra(extra_root)

            if extra_root.tag == "body":
                worldbody.append(extra_root)
                continue

            extra_worldbody = extra_root.find("worldbody")
            if extra_worldbody is not None:
                for child in list(extra_worldbody):
                    if child.tag == "body":
                        worldbody.append(child)
                continue

            for child in list(extra_root):
                if child.tag == "body":
                    worldbody.append(child)

        # 合并完成后再统一补齐碰撞 bit（包含 extras）
        ensure_collision_bits(base_root)

        base_dir = Path(self.model_path).parent
        tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".xml", dir=base_dir, prefix="combined_")
        self._tmp_combined_path = Path(tmp_file.name)
        try:
            xml_bytes = ET.tostring(base_root, encoding="utf-8", method="xml")
            tmp_file.write(xml_bytes)
        finally:
            tmp_file.close()

        return mj.MjModel.from_xml_path(str(self._tmp_combined_path))

    def reset(self):
        if self.model is None or self.data is None:
            return
        self.mj.mj_resetData(self.model, self.data)
        if self.model.nu > 0:
            self.data.ctrl[:] = 0
        self.mj.mj_forward(self.model, self.data)

    def apply_joint_positions(self, joint_positions_rad: Iterable[float]):
        if self.data is None:
            return
        targets = clamp_to_limits(joint_positions_rad)

        # 用 position actuator 驱动（而不是直接写 qpos），才能产生接触/动力学响应
        for idx, name in enumerate(JOINT_ORDER):
            act_id = self.actuator_id[name]
            target = float(targets[idx])
            if bool(self.model.actuator_ctrllimited[act_id]):
                lo, hi = self.model.actuator_ctrlrange[act_id]
                target = float(np.clip(target, lo, hi))
            self.data.ctrl[act_id] = target

        for _ in range(self._resolved_steps_per_control):
            self.mj.mj_step(self.model, self.data)

        if self.viewer is not None:
            self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
        self.viewer = None
        self.model = None
        self.data = None
        self.mj = None
        if self._tmp_combined_path and self._tmp_combined_path.exists():
            try:
                os.remove(self._tmp_combined_path)
            except Exception:
                pass
        self._tmp_combined_path = None

    def get_end_effector_wrench(self, body_name: str = "gripper") -> np.ndarray:
        """返回指定 body 的外力/力矩 6D（世界系，近似）。

        说明：
        - `data.cfrc_ext` 在某些情况下可能一直为 0（与 MuJoCo 版本/管线有关）。
        - 为了让“代理力反馈”可用，这里在 `cfrc_ext` 近似为 0 时，回退到从 `data.contact`
          中提取接触力并累加（只返回力的模长，方向/力矩置 0）。
        """
        if self.data is None or self.model is None or self.mj is None:
            return np.zeros(6, dtype=np.float32)
        mj = self.mj
        body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return np.zeros(6, dtype=np.float32)
        wrench = self.data.cfrc_ext[body_id].copy()
        if float(np.linalg.norm(wrench[:3])) > 1e-8:
            return wrench

        # Fallback: sum contact force magnitudes involving geoms in this body.
        total_contact_force = 0.0
        for i in range(int(self.data.ncon)):
            c = self.data.contact[i]
            b1 = int(self.model.geom_bodyid[int(c.geom1)])
            b2 = int(self.model.geom_bodyid[int(c.geom2)])
            if b1 != body_id and b2 != body_id:
                continue
            f = np.zeros(6, dtype=np.float64)
            mj.mj_contactForce(self.model, self.data, i, f)
            total_contact_force += float(np.linalg.norm(f[:3]))

        out = np.zeros(6, dtype=np.float32)
        out[0] = float(total_contact_force)
        return out
