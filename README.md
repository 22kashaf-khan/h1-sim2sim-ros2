# h1-sim2sim-ros2

Deploying a Unitree H1 locomotion policy trained in NVIDIA Isaac Lab through a
C++ ROS2 node, controlling the robot in MuJoCo (sim-to-sim transfer).

**Status:** work in progress — weeks 1–3 of 5 done.

## Pipeline

Isaac Lab (training) → ONNX export → C++ ROS2 node (ONNX Runtime) → MuJoCo bridge → H1 in MuJoCo

## Progress

- [x] Week 1: `h1_policy` ROS2 node skeleton (C++): 50 Hz control loop, `sensor_msgs/JointState`
      in and out (mapped by joint name), stale-data watchdog.
- [x] Week 2: policy export and interface. ONNX model, deployment config generated
      from Isaac Lab, reference test cases.
- [x] Week 3: C++ inference. ONNX Runtime in a ROS-free core library (`h1_policy_core`),
      observation builder, node publishes policy targets at 50 Hz. C++ reproduces the
      Python reference outputs exactly (max difference 0).
- [ ] Week 4: MuJoCo bridge, lockstep sim-to-sim
- [ ] Week 5: real-time mode, latency/jitter measurements

## Node interface (`h1_policy/policy_node`)

| Topic | Type | Direction |
|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | in (position + velocity, mapped by name) |
| `imu` | `sensor_msgs/Imu` | in (orientation, angular velocity, body frame) |
| `odom` | `nav_msgs/Odometry` | in (linear velocity, body frame) |
| `cmd_vel` | `geometry_msgs/Twist` | in (optional; defaults to zero) |
| `joint_command` | `sensor_msgs/JointState` | out (`position` = target angles, policy joint order) |

Parameter: `policy_dir` (required), the folder containing `policy.onnx` and `deploy.yaml`.
If `joint_states`, `imu` or `odom` is older than 100 ms, the node stops publishing.

## Build and run

Requirements: Ubuntu 24.04, ROS2 Jazzy, yaml-cpp (installed with ROS2), and the
ONNX Runtime 1.31.0 C++ release (`onnxruntime-linux-x64-1.31.0.tgz` from the
[ONNX Runtime releases](https://github.com/microsoft/onnxruntime/releases)), unpacked to
`~/libs/onnxruntime-linux-x64-1.31.0` (or pass `-DONNXRUNTIME_ROOT=<path>` to CMake).

```bash
cd ~/ros2_ws
colcon build --packages-select h1_policy
colcon test --packages-select h1_policy
source install/setup.bash
ros2 run h1_policy policy_node --ros-args -p policy_dir:=<repo>/h1_policy/policies/h1_flat_baseline
```

## Policy

Files in `h1_policy/policies/h1_flat_baseline/`:

| File | Content | Produced by |
|---|---|---|
| `policy.onnx` | Actor network, input `obs` [1, 69] → output `actions` [1, 19] | Isaac Lab `play.py` |
| `deploy.yaml` | Joint order, default pose, PD gains, timing, observation layout | `tools/isaaclab/print_joint_info.py` |
| `test_cases.yaml` | 8 reference observation/action pairs for testing the C++ inference | `tools/policy/make_test_cases.py` |
| `env.yaml`, `agent.yaml` | Original Isaac Lab training configuration | Isaac Lab (unmodified copies) |

**Provenance**

- Task `Isaac-Velocity-Flat-H1-v0`, own training run: 1500 PPO iterations,
  MLP 69 → 128 → 128 → 128 → 19 (ELU), no observation normalization.
- Exported on 2026-08-12 with Isaac Lab's `play.py` (default checkpoint selection,
  which picked `model_1499.pt`, the final checkpoint; it existed before the export).
- `policy.onnx` SHA-256: `05542935ae7758c2c9466c941ffd0572be22068097dbabbc53584c9907536af1`
- Software: Isaac Lab v2.3.2, Isaac Sim 5.1.0, PyTorch 2.7.0, official `rsl-rl-lib`;
  deployment uses ONNX Runtime 1.31.0.
- Hardware: Intel Core Ultra 9 285HX, NVIDIA RTX PRO 4000 Blackwell (16 GB), Windows 11 + WSL2.

**Reference policy (not included):** NVIDIA's pretrained `Isaac-Velocity-Flat-H1-v0`
policy has the same interface and is used as a known-good reference. Obtain it with
Isaac Lab's `play.py --use_pretrained_checkpoint`.

## Policy interface

Observation (69 values, no scaling, no clipping, no noise at deployment):

| Index | Term | Meaning |
|---|---|---|
| 0–2 | `base_lin_vel` | Base linear velocity, body frame |
| 3–5 | `base_ang_vel` | Base angular velocity, body frame |
| 6–8 | `projected_gravity` | Gravity direction in the body frame |
| 9–11 | `velocity_commands` | Commanded (vx, vy, yaw rate) |
| 12–30 | `joint_pos` | Joint position minus default |
| 31–49 | `joint_vel` | Joint velocity |
| 50–68 | `actions` | Previous raw policy output |

Action: `target = default_joint_pos + 0.5 × action` (rad), no clipping.
Joints use Isaac Lab's breadth-first order (see `deploy.yaml`), so mapping to other
simulators is done by joint name, never by index. Control period 20 ms (5 ms physics × 4).

## Regenerating the generated files

```bash
# deploy.yaml (Windows, Isaac Lab environment)
python tools/isaaclab/print_joint_info.py --output h1_policy/policies/h1_flat_baseline/deploy.yaml

# test_cases.yaml (Python venv with onnx, onnxruntime, numpy, pyyaml)
python tools/policy/make_test_cases.py h1_policy/policies/h1_flat_baseline
```

## Notes

- Under WSL2, the 50 Hz loop shows ~0.5 s stalls at a regular interval of about
  33 s (49.3 Hz average, max gap 0.525 s over ~106 s), occasionally longer (~1.4 s)
  under extra load. Input and output stall together, which points to the WSL2 VM
  pausing rather than the node. The stale-data watchdog catches them and stops
  commanding. To be quantified and compared with bare-metal Linux in week 5.
- Given a perfectly still default-pose observation, the policy commands an
  asymmetric knee/ankle bend, suggesting it steps in place rather than standing
  still. To be checked in closed loop in MuJoCo (week 4).
- ONNX Runtime 1.31.0's bundled CMake config points to `lib64/` and
  `include/onnxruntime/`, which don't exist in the release archive, so
  `CMakeLists.txt` defines the `onnxruntime::onnxruntime` target itself.