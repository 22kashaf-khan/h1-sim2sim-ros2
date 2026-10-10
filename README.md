# h1-sim2sim-ros2

Deploying a Unitree H1 locomotion policy trained in NVIDIA Isaac Lab through a
C++ ROS2 node, controlling the robot in MuJoCo (sim-to-sim transfer).

**Status:** work in progress — week 1 of 5 done.

## Progress

- [x] Week 1: `h1_policy` ROS2 node skeleton (C++). Subscribes to `joint_states`,
      runs a 50 Hz control loop, publishes hold-position targets on `joint_command`
      (`sensor_msgs/JointState`, mapped by joint name), with a 100 ms stale-data watchdog.
- [ ] Week 2: ONNX export of the Isaac Lab policy + parity check
- [ ] Week 3: ONNX Runtime inference in the C++ node
- [ ] Week 4: MuJoCo bridge, lockstep sim-to-sim
- [ ] Week 5: real-time mode, latency/jitter measurements

## Notes

- Under WSL2, the 50 Hz loop shows ~0.5 s stalls at a regular interval of about
  33 s (49.3 Hz average, max gap 0.525 s over ~106 s). Input and output stall
  together, and some stalls are invisible to the node itself, which points to the
  whole WSL2 VM pausing rather than the node. The stale-data watchdog catches most
  of them. To be quantified and compared with bare-metal Linux in week 5.

## Planned pipeline

Isaac Lab (training) → ONNX export → C++ ROS2 node (ONNX Runtime) → MuJoCo bridge → H1 in MuJoCo

## Environment

- Ubuntu 24.04 (WSL2), ROS2 Jazzy
