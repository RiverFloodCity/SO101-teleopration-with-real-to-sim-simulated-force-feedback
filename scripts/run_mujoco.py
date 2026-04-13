from __future__ import annotations

import argparse
import os
from pathlib import Path

import sys

# 确保仓库根目录在 sys.path，便于直接 `python scripts/run_mujoco.py`
ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))


def parse_args():
    parser = argparse.ArgumentParser(description="SO101 Leader 遥操作 -> Mujoco 仿真 SO101 Follower")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="SO101 Leader 串口")
    parser.add_argument(
        "--model-path",
        type=str,
        default=str(Path(__file__).resolve().parents[1] / "assets" / "models" / "so101_new_calib.xml"),
        help="Mujoco 模型 XML 路径（默认使用 so101_new_calib.xml）",
    )
    parser.add_argument("--hz", type=float, default=60.0, help="控制频率")
    parser.add_argument("--no-render", action="store_true", help="禁用渲染（无窗口）")
    parser.add_argument("--recalibrate", action="store_true", help="强制重新校准 leader")
    parser.add_argument(
        "--lowpass-alpha",
        type=float,
        default=None,
        help="可选：低通滤波系数 (0~1，越小越平滑)。不填则关闭滤波。",
    )
    parser.add_argument(
        "--extras",
        type=str,
        default="",
        help="附加物体列表（逗号分隔，位于 assets/models/objects/ 下的 xml 文件名，不含路径）。默认空（不加载）。如 ground_plane,block_small",
    )
    parser.add_argument(
        "--forcefeedback",
        type=str,
        choices=["on", "off"],
        default="off",
        help="是否开启仿真力反馈推回 leader（安全起见默认 off）",
    )
    parser.add_argument(
        "--ff-scale",
        type=float,
        default=0.01,
        help="力反馈缩放系数，越大推回越强",
    )
    parser.add_argument(
        "--ff-max-deg",
        type=float,
        default=5.0,
        help="力反馈最大单次偏置（leader 归一化单位，非度；例如 gripper 0~100）",
    )
    parser.add_argument(
        "--ff-lowpass-alpha",
        type=str,
        default="0.2",
        help="力反馈低通系数（如 0.2）；传 none/off/disable 关闭滤波",
    )
    parser.add_argument(
        "--ff-contact-thresh",
        type=float,
        default=0.2,
        help="力反馈触发阈值（N），低于此值不推回",
    )
    parser.add_argument(
        "--ff-bodies",
        type=str,
        default="moving_jaw_so101_v1",
        help="参与力反馈的 body 名称列表（逗号分隔）。默认仅活动爪指（gripper 舵机驱动的那一侧）",
    )
    parser.add_argument(
        "--ff-direction",
        type=str,
        choices=["open", "close"],
        default="open",
        help="力反馈方向：open=推向张开（默认，仅在关夹爪时抵抗），close=推向闭合",
    )
    parser.add_argument(
        "--ff-deadband",
        type=float,
        default=0.02,
        help="夹爪动作死区（leader 归一化单位），小于该变化量时不施加反馈，避免持续推偏",
    )
    parser.add_argument(
        "--ff-torque-fraction",
        type=float,
        default=0.3,
        help="力反馈期间将 leader 夹爪 Torque_Limit 按比例缩放（0~1），越小越“软”越不容易锁死",
    )
    parser.add_argument(
        "--ff-debug",
        action="store_true",
        help="打印力反馈调试信息（力/偏置/运动方向/扭矩开关），用于确认链路是否生效",
    )
    return parser.parse_args()


def _parse_optional_float(value: str):
    v = value.strip().lower()
    if v in {"none", "null", "off", "disable", "disabled", ""}:
        return None
    return float(value)


def main():
    args = parse_args()

    # 延迟导入：允许在未安装依赖时也能 `--help`/解析参数
    from hardware.leader.so101_leader import SO101Leader
    from sim.mujoco.so101_env import MujocoSO101Env
    from teleop.controller import run_teleop

    model_path = os.path.abspath(args.model_path)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"未找到模型文件: {model_path}")

    extras_list = [x for x in args.extras.split(",") if x.strip()]
    extras_paths = [
        os.path.join(ROOT_DIR, "assets", "models", "objects", f"{name.strip()}.xml") for name in extras_list
    ]
    missing_extras = [p for p in extras_paths if not os.path.exists(p)]
    if missing_extras:
        raise FileNotFoundError(f"未找到 extras 文件: {missing_extras}")

    ff_lowpass_alpha = _parse_optional_float(args.ff_lowpass_alpha)

    sim_env = None
    leader = None
    try:
        # 先连接硬件：若硬件初始化失败，不要留下 mujoco viewer/线程未清理，避免异常退出时 core dump
        leader = SO101Leader(port=args.port, recalibrate=args.recalibrate, connect_on_init=False)
        leader.connect()

        sim_env = MujocoSO101Env(
            model_path=model_path,
            render=not args.no_render,
            extras=extras_paths,
            control_dt=1.0 / float(args.hz),
        )
        sim_env.connect()
        sim_env.reset()

        print(f"已连接 leader({args.port})，仿真模型: {model_path}")
        if args.lowpass_alpha is not None:
            print(f"启用低通滤波 alpha={args.lowpass_alpha}")

        run_teleop(
            leader,
            sim_env,
            hz=args.hz,
            lowpass_alpha=args.lowpass_alpha,
            force_feedback=args.forcefeedback == "on",
            ff_scale=args.ff_scale,
            ff_max_offset=args.ff_max_deg,
            ff_lowpass_alpha=ff_lowpass_alpha,
            ff_contact_thresh=args.ff_contact_thresh,
            ff_bodies=tuple([x.strip() for x in args.ff_bodies.split(",") if x.strip()]),
            ff_direction=args.ff_direction,
            ff_deadband=args.ff_deadband,
            ff_torque_fraction=args.ff_torque_fraction,
            ff_debug=args.ff_debug,
        )
    except KeyboardInterrupt:
        print("收到中断，退出。")
    except RuntimeError as exc:
        msg = str(exc)
        if "Input voltage error" in msg or "input voltage error" in msg:
            print(
                "检测到电机回报 [Input voltage error]：通常表示舵机供电电压异常（偏低/掉电/供电不稳）。\n"
                "建议检查：电源/电池电压、接线/地线、转接板供电是否足够；特别是 id=6（gripper）那路。"
            )
        raise
    finally:
        try:
            if leader is not None:
                try:
                    leader.disable_torque()
                except Exception:
                    pass
                leader.disconnect()
        except Exception:
            pass
        try:
            if sim_env is not None:
                sim_env.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
