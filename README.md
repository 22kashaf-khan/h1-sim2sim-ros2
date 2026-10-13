# h1-sim2sim-ros2

Deploying a Unitree H1 locomotion policy trained in NVIDIA Isaac Lab through a
C++ ROS2 node, controlling the robot in MuJoCo (sim-to-sim transfer).

**Demo:** H1 walking forward, turning and stopping in MuJoCo, controlled by the C++ ROS2 policy node (run A, real time).

https://github.com/user-attachments/assets/98d2dcd2-9b20-4929-a291-674341b8378c



**Status:** the policy stands and walks in MuJoCo, driven by the C++ ROS2 nodes.
Measurements (tracking error, robustness, latency) are in progress.

## Pipeline

Isaac Lab (training) → ONNX export → C++ ROS2 policy node (ONNX Runtime) ⇄ C++ MuJoCo bridge → H1 in MuJoCo

## Results so far

Policy `Isaac-Velocity-Flat-H1-v0` (own run), MuJoCo 3.15.0, H1 from MuJoCo Menagerie,
real-time mode under WSL2. Single runs, informal measurements; to be quantified.

| | Run A: Isaac Lab torque limits | Run B: Menagerie torque limits |
|---|---|---|
| Torque limits | 300 Nm, ankles 100 Nm (as in training) | 18–300 Nm (ankles 40, elbows 18) |
| Stands | ✅ ~0.90 m torso height | ✅ ~0.91 m |
| Walks at 0.5 m/s command | ✅ 14.5 m forward | ✅ 17.1 m in ~35 s (≈ 0.49 m/s) |
| Sideways drift | 0.2 m (~0.8°) | 2.8 m (~9°) |

The policy transfers to a different physics engine and tolerates much weaker,
more realistic actuators than it was trained with; the weaker actuators mainly
show up as heading drift.

## Progress

- [x] Week 1: ROS2 node skeleton (C++): 50 Hz loop, joint states in/out by name, stale-data watchdog.
- [x] Week 2: policy export and interface: ONNX model, deployment config generated from Isaac Lab, reference test cases.
- [x] Week 3: C++ inference: ONNX Runtime in a ROS-free core library; C++ reproduces Python exactly.
- [x] Week 4: MuJoCo bridge (C++): explicit PD per physics step, launch file, H1 stands and walks.
- [ ] Lockstep mode with simulation time (repeatable runs)
- [ ] Week 5: measured tracking error, drift, robustness to pushes, latency/jitter, video

## Packages

| Package | Content |
|---|---|
| `h1_policy` | Policy node + ROS-free core library (`DeployConfig`, `ObservationBuilder`, `OnnxPolicy`), policy files |
| `h1_mujoco_bridge` | MuJoCo bridge node + ROS-free core library (`H1Sim`: PD control, body-frame state) |
| `h1_bringup` | `sim2sim.launch.py`: starts both nodes |

Topics: `joint_states`, `imu`, `odom` (bridge → policy), `joint_command` (policy → bridge),
`cmd_vel` (optional velocity command). All joint data is mapped by name.
The bridge holds the simulation still until the policy's first command.

## Build and run

Requirements: Ubuntu 24.04, ROS2 Jazzy, yaml-cpp (installed with ROS2), and:

| Dependency | Version | Default location (override) |
|---|---|---|
| ONNX Runtime C++ (`onnxruntime-linux-x64-1.31.0.tgz`) | 1.31.0 | `~/libs/onnxruntime-linux-x64-1.31.0` (`-DONNXRUNTIME_ROOT=`) |
| MuJoCo (`mujoco-3.15.0-linux-x86_64.tar.gz`) | 3.15.0 | `~/libs/mujoco-3.15.0` (`-DMUJOCO_ROOT=`) |
| MuJoCo Menagerie, `unitree_h1` | commit `0059d4335f8156206f63a35662313385f7ad6d74` | `~/libs/mujoco_menagerie` (`-DH1_MODEL_DIR=`, launch arg `scene:=`) |

```bash
cd ~/ros2_ws
colcon build --packages-select h1_policy h1_mujoco_bridge h1_bringup
colcon test --packages-select h1_policy h1_mujoco_bridge
source install/setup.bash

ros2 launch h1_bringup sim2sim.launch.py                        # run A
ros2 launch h1_bringup sim2sim.launch.py torque_limits:=model   # run B

ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## Sim-to-sim matching

| Item | Isaac Lab (training) | MuJoCo Menagerie | Handled by |
|---|---|---|---|
| Joint order | Breadth-first | Depth-first (2 of 19 match) | Mapping by joint name everywhere |
| Actuators | Implicit PD | Torque motors | Explicit PD in the bridge, every 5 ms step |
| Kp / Kd | 20–200 / 4–10 | — | From `deploy.yaml` |
| Physics step / policy rate | 5 ms × 4 | 2 ms default | Bridge sets 5 ms × 4 |
| Torque limits | 300 / 100 Nm | 18–300 Nm | Launch argument (`isaac` / `model`) |
| Start height | 1.05 m (Isaac root) | Feet ~7 cm in the air at 1.05 m | Bridge lowers the robot until the feet touch |
| Base velocity frame | Body | Linear velocity in world frame | Bridge rotates it into the body frame |
| Quaternion order | (w, x, y, z) | (w, x, y, z); ROS uses x, y, z, w fields | Explicit conversion at both ROS boundaries |
| Joint damping / armature | From the USD file | 1 / 0.1 | Kept as in the model (not yet compared) |
| Foot contact | Collision meshes | 3 capsules per foot | Inherent engine difference |

## Policy

Files in `h1_policy/policies/h1_flat_baseline/`:

| File | Content | Produced by |
|---|---|---|
| `policy.onnx` | Actor network, `obs` [1, 69] → `actions` [1, 19] | Isaac Lab `play.py` |
| `deploy.yaml` | Joint order, default pose, PD gains, timing, observation layout | `tools/isaaclab/print_joint_info.py` |
| `test_cases.yaml` | 8 reference observation/action pairs | `tools/policy/make_test_cases.py` |
| `env.yaml`, `agent.yaml` | Original Isaac Lab training configuration | Isaac Lab (unmodified copies) |

**Provenance**

- Task `Isaac-Velocity-Flat-H1-v0`, own training run: 1500 PPO iterations,
  MLP 69 → 128 → 128 → 128 → 19 (ELU), no observation normalization.
- Exported on 2026-08-12 with Isaac Lab's `play.py` (default checkpoint selection → `model_1499.pt`).
- `policy.onnx` SHA-256: `05542935ae7758c2c9466c941ffd0572be22068097dbabbc53584c9907536af1`
- Software: Isaac Lab v2.3.2, Isaac Sim 5.1.0, PyTorch 2.7.0, official `rsl-rl-lib`.
- Hardware: Intel Core Ultra 9 285HX, NVIDIA RTX PRO 4000 Blackwell (16 GB), Windows 11 + WSL2.

**Reference policy (not included):** NVIDIA's pretrained `Isaac-Velocity-Flat-H1-v0`
(same interface). Obtain it with Isaac Lab's `play.py --use_pretrained_checkpoint`.

**Observation** (69 values, no scaling, clipping or noise at deployment): base linear velocity (0–2),
base angular velocity (3–5), projected gravity (6–8), velocity command (9–11),
joint position minus default (12–30), joint velocity (31–49), previous raw action (50–68),
all in body frame and policy joint order.
**Action:** `target = default_joint_pos + 0.5 × action` (rad), no clipping.

## Notes

- **Start conditions matter.** The first closed-loop run fell: the robot started ~7 cm in
  the air (Isaac Lab's start height doesn't fit the Menagerie model) and the simulation
  ran before the policy was connected (ROS2 discovery takes ~1 s; PD alone holds the
  pose for ~1 s). Starting on the ground and waiting for the first command fixed it,
  without any change to the policy or observation code.
- **WSL2 stalls.** The 50 Hz loop shows ~0.5–1.4 s stalls at a regular interval of
  about 33 s. Input and output stall together, and the robot survives 1.4 s gaps while
  walking, which points to the whole WSL2 VM pausing (simulation included). The
  stale-data watchdog stops commanding during stalls. To be quantified.
- ONNX Runtime 1.31.0's bundled CMake config points to `lib64/` and `include/onnxruntime/`,
  which don't exist in the release archive; the same applies to MuJoCo, which ships
  no CMake config. Both are located by `find_library`/`find_path` instead.
