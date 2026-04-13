# real_to_sim_teleop

一个面向 SO101 的 real-to-sim 遥操作项目：使用真实 **SO101 Leader** 机械臂，实时控制 **Mujoco** 中的 **SO101 Follower** 机械臂。

项目的核心特性不是单纯“跟手控制”，而是提供了一个可调的 **模拟力反馈（simulated force feedback）** 链路：
- 读取 Mujoco 中 follower 夹爪与环境的接触/受力信息；
- 将接触强度映射为 leader 侧的夹爪偏置与阻力变化；
- 在没有真实双向力控硬件闭环的前提下，给操作者提供“碰到物体了”“继续夹会有阻力”的操作感。

这使它适合用来验证以下场景：
- SO101 的 reality-to-sim 遥操作链路；
- 面向抓取/接触任务的人在环控制；
- 低成本“代理力反馈”交互方案；
- 后续 sim-to-real、数据采集或策略验证前的功能原型。

## 主要功能
- **SO101 Leader -> SO101 Follower 的实时遥操作**：读取真实 leader 臂各关节状态，并映射到 Mujoco 中的 follower 臂。
- **模拟力反馈**：将仿真中的接触信息转成 leader 侧可感知的“推回 / 阻力”效果，是本项目当前最重要的特性。
- **可配置的夹爪反馈策略**：支持调节反馈强度、触发阈值、作用方向、死区和 torque fraction。
- **模块化结构**：硬件接入、关节映射、仿真环境、控制循环彼此解耦，便于替换仿真后端或扩展控制逻辑。

## 参考与来源
- 本项目并非从零开始实现，部分底层电机总线代码、SO101 电机/关节限制设定与接口设计参考并改写自 `LeRobot`。
- 仓库中保留了相关源码文件原有的版权与许可证头；如果你继续分发或公开该项目，建议同时保留这些声明，并在仓库根目录补充对应的许可证/第三方说明文件。

## 架构示意（文件/层次）
```
real_to_sim_teleop/
├─ README.md
├─ requirements.txt               # 依赖
├─ configs/                       # 预留：配置/参数（当前未强依赖）
├─ assets/
│  └─ models/
│     ├─ so101_new_calib.xml      # 默认模型（引用 assets/ 与 objects/）
│     ├─ assets/                  # 模型 mesh（.stl）
│     │  ├─ base_so101_v2.stl
│     │  ├─ moving_jaw_so101_v1.stl
│     │  └─ ...
│     └─ objects/                 # extras：可选附加物体（通过 --extras 加载）
│        ├─ ground_plane.xml
│        ├─ block_small.xml
│        └─ ...
├─ hardware/                      # 真实设备层
│  ├─ leader/so101_leader.py      # SO101 Leader 封装（校准/读数/连接）
│  └─ common/                     # 通用底座
│     ├─ errors.py, utils.py
│     └─ motors/feetech/          # Feetech 总线实现（scservo_sdk）
│        ├─ feetech.py
│        ├─ tables.py, encoding_utils.py
│        └─ ...
├─ mapping/
│  ├─ so101_mapping.py            # 电机↔仿真关节映射、限位、低通
│  └─ force_mapping.py            # 仿真接触力 -> leader 偏置（代理力反馈）
├─ sim/
│  ├─ base/environment.py         # 仿真接口抽象
│  └─ mujoco/so101_env.py         # Mujoco 后端实现
├─ teleop/
│  └─ controller.py               # 主循环（读硬件→映射→仿真）
├─ scripts/
│  └─ run_mujoco.py               # 入口脚本，选择模型/端口/频率/滤波
└─ .gitignore
```

## 使用
```bash
# 新建环境-进入环境（建议Python 3.10~3.12)

# 安装依赖
pip install -r requirements.txt

# 连接机械臂后，给串口权限
ls /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0

# 运行指令示例
python scripts/run_mujoco.py --port=/dev/ttyACM0 --hz=60 \
  --extras=ground_plane,block_small \
  --forcefeedback=on --ff-bodies moving_jaw_so101_v1 --ff-scale 0.01 --ff-max-deg 5.0 --ff-contact-thresh 0.2 --ff-direction open --ff-deadband 0.02 --ff-torque-fraction 1.0

# 调试力反馈链路（每0.5秒打印实时力反馈状态。推荐先把阈值设为 0，确认确实读到了接触力/触发了 torque）
python scripts/run_mujoco.py --port=/dev/ttyACM0 --hz=60 \
  --extras=ground_plane,block_small \
  --forcefeedback=on --ff-debug --ff-contact-thresh 0

# 可选：
#   --model-path assets/models/so101_new_calib.xml  # 默认模型（可显式指定）
#   --extras=ground_plane,block_small           # 选择附加物体（assets/models/objects 下；默认不加载）
#   --no-render                                 # 关闭窗口（也可用于规避 Wayland/GLFW 的窗口提示）
#   --recalibrate                               # 强制重跑校准（会进入交互式校准流程）
#   --lowpass-alpha 0.2                         # 启用一阶低通平滑
#   --forcefeedback=on --ff-scale 0.01 --ff-contact-thresh 0.2  # 开启仿真力反馈推回 leader（仅接触时触发）
#   --ff-lowpass-alpha none                     # 关闭力反馈低通
#   --ff-bodies moving_jaw_so101_v1              # 参与力反馈的 body（默认活动爪指；只在该爪指接触时反馈）
#   --ff-direction open                          # open=推向张开（默认，用于抵抗关夹爪），close=推向闭合（用于抵抗开夹爪）
#   --ff-deadband 0.02                           # 夹爪运动死区（leader 归一化单位），小于该变化量不触发反馈
#   --ff-torque-fraction 0.3                     # 力反馈期间缩放夹爪 Torque_Limit（越小越软）
#   --ff-debug                                   # 打印力反馈调试信息，确认是否触发/偏置是否为 0
```

## 扩展点
- 加滤波/安全层：在 `mapping` 中添加新的处理器，并在 `controller` 中串联。
- 多仿真后端：实现 `SimInterface` 子类（如 `sim/isaac/so101_env.py`），入口脚本按需切换。
- 录制/回放：在 `controller` 循环中插入记录器或策略推理模块，不影响硬件/仿真实现。
