# h1-sim2sim-ros2

Deploying a Unitree H1 locomotion policy trained in NVIDIA Isaac Lab through a
C++ ROS2 node, controlling the robot in MuJoCo (sim-to-sim transfer).

**Status:** work in progress — weeks 1–2 of 5 done.

## Pipeline

Isaac Lab (training) → ONNX export → C++ ROS2 node (ONNX Runtime) → MuJoCo bridge → H1 in MuJoCo

## Progress

- [x] Week 1: `h1_policy` ROS2 node skeleton (C++). Subscribes to `joint_states`,
      runs a 50 Hz control loop, publishes hold-position targets on `joint_command`
      (`sensor_msgs/JointState`, mapped by joint name), with a 100 ms stale-data watchdog.
- [x] Week 2: policy export and interface. ONNX model, deployment config generated
      from Isaac Lab, reference test cases for the C++ inference.
- [ ] Week 3: ONNX Runtime inference in the C++ node
- [ ] Week 4: MuJoCo bridge, lockstep sim-to-sim
- [ ] Week 5: real-time mode, latency/jitter measurements

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
- Software: Isaac Lab v2.3.2, Isaac Sim 5.1.0, PyTorch 2.7.0, official `rsl-rl-lib`.
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

## Environment

- Ubuntu 24.04 (WSL2), ROS2 Jazzy

## Notes

- Under WSL2, the 50 Hz loop shows ~0.5 s stalls at a regular interval of about
  33 s (49.3 Hz average, max gap 0.525 s over ~106 s). Input and output stall
  together, and some stalls are invisible to the node itself, which points to the
  whole WSL2 VM pausing rather than the node. The stale-data watchdog catches most
  of them. To be quantified and compared with bare-metal Linux in week 5.
- Given a perfectly still default-pose observation, the policy commands a deeper,
  asymmetric knee/ankle bend (knee targets ~1.2–1.4 rad vs 0.79 default), suggesting
  it steps in place rather than standing still. To be checked in MuJoCo (week 4).