# real_to_sim_teleop

A real-to-sim teleoperation project for SO101: a physical **SO101 Leader** arm
controls an **SO101 Follower** arm in **MuJoCo** in real time.

The main feature of this project is not just joint mirroring, but a tunable
**simulated force feedback** pipeline:
- read contact / interaction signals from the follower gripper in MuJoCo
- map contact intensity to gripper-side offset and resistance on the leader
- give the operator a usable sense of "the gripper touched something" or
  "closing further now feels resisted", even without a full bidirectional
  hardware force-control loop

This makes the project useful for:
- SO101 reality-to-sim teleoperation experiments
- human-in-the-loop grasping and contact tasks
- low-cost proxy force-feedback interaction
- prototyping before sim-to-real transfer, data collection, or policy testing

## Main Features
- **Real-time SO101 Leader -> SO101 Follower teleoperation**: reads joint states
  from the physical leader arm and maps them to the MuJoCo follower arm.
- **Simulated force feedback**: converts contact information in simulation into
  a perceivable push-back / resistance effect on the leader side. This is the
  primary feature of the project.
- **Configurable gripper feedback policy**: supports adjustable feedback scale,
  trigger threshold, direction, deadband, and torque fraction.
- **Modular structure**: hardware access, joint mapping, simulation backend, and
  control loop are decoupled, making it easier to extend or swap components.

## Reference and Upstream Basis
- This project was not implemented from scratch. Parts of the low-level motor
  bus code, as well as some SO101 motor / joint limit conventions and interface
  design, were adapted from `LeRobot`.
- The original copyright and license headers have been kept in the relevant
  source files. If you redistribute this repository, you should keep those
  notices and include top-level license / third-party notice files as well.

## Project Structure
```
real_to_sim_teleop/
├─ README.md
├─ requirements.txt               # dependencies
├─ configs/                       # reserved for configs / parameters
├─ assets/
│  └─ models/
│     ├─ so101_new_calib.xml      # default model, referencing assets/ and objects/
│     ├─ assets/                  # mesh assets (.stl)
│     │  ├─ base_so101_v2.stl
│     │  ├─ moving_jaw_so101_v1.stl
│     │  └─ ...
│     └─ objects/                 # optional extra objects loaded via --extras
│        ├─ ground_plane.xml
│        ├─ block_small.xml
│        └─ ...
├─ hardware/                      # real hardware layer
│  ├─ leader/so101_leader.py      # SO101 Leader wrapper (calibration / read / connect)
│  └─ common/                     # shared utilities
│     ├─ errors.py, utils.py
│     └─ motors/feetech/          # Feetech motor bus implementation (scservo_sdk)
│        ├─ feetech.py
│        ├─ tables.py, encoding_utils.py
│        └─ ...
├─ mapping/
│  ├─ so101_mapping.py            # motor <-> sim joint mapping, limits, low-pass
│  └─ force_mapping.py            # sim contact force -> leader offset (proxy force feedback)
├─ sim/
│  ├─ base/environment.py         # abstract simulation interface
│  └─ mujoco/so101_env.py         # MuJoCo backend
├─ teleop/
│  └─ controller.py               # main loop (read hardware -> map -> simulate)
├─ scripts/
│  └─ run_mujoco.py               # entry point, selects model / port / rate / filters
└─ .gitignore
```

## Usage
```bash
# Create and activate an environment (Python 3.10-3.12 recommended)

# Install dependencies
pip install -r requirements.txt

# After connecting the robot arm, grant serial port access
ls /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0

# Example run command
python scripts/run_mujoco.py --port=/dev/ttyACM0 --hz=60 \
  --extras=ground_plane,block_small \
  --forcefeedback=on --ff-bodies moving_jaw_so101_v1 --ff-scale 0.01 --ff-max-deg 5.0 --ff-contact-thresh 0.2 --ff-direction open --ff-deadband 0.02 --ff-torque-fraction 1.0

# Debug the force-feedback path (prints force-feedback status every 0.5s).
# It is recommended to first set the threshold to 0 to verify that contact
# force is actually detected and torque logic is being triggered.
python scripts/run_mujoco.py --port=/dev/ttyACM0 --hz=60 \
  --extras=ground_plane,block_small \
  --forcefeedback=on --ff-debug --ff-contact-thresh 0

# Optional:
#   --model-path assets/models/so101_new_calib.xml   # default model (can be set explicitly)
#   --extras=ground_plane,block_small                # extra objects under assets/models/objects; disabled by default
#   --no-render                                      # disable the viewer window
#   --recalibrate                                    # force interactive recalibration
#   --lowpass-alpha 0.2                              # enable first-order low-pass smoothing
#   --forcefeedback=on --ff-scale 0.01 --ff-contact-thresh 0.2
#                                                   # enable simulated force feedback on the leader, only when contact is detected
#   --ff-lowpass-alpha none                          # disable force-feedback low-pass filtering
#   --ff-bodies moving_jaw_so101_v1                  # bodies used for feedback; default is the active finger pad
#   --ff-direction open                              # open = push toward opening (default), close = push toward closing
#   --ff-deadband 0.02                               # gripper motion deadband in normalized leader units
#   --ff-torque-fraction 0.3                         # scale gripper Torque_Limit during feedback
#   --ff-debug                                       # print debugging information for force feedback
```

## Extension Points
- Add filtering or safety logic: implement new processors in `mapping` and
  chain them in `controller`.
- Add more simulation backends: implement another `SimInterface` subclass, such
  as `sim/isaac/so101_env.py`, and switch it from the entry script.
- Add recording / replay: insert a logger or policy module into the controller
  loop without changing the hardware or simulation implementations.
