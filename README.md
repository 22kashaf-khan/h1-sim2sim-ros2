# h1-sim2sim-ros2

Deploying a Unitree H1 locomotion policy trained in NVIDIA Isaac Lab through a
C++ ROS2 node, controlling the robot in MuJoCo (sim-to-sim transfer).

**Status:** work in progress — environment setup.

## Planned pipeline

Isaac Lab (training) → ONNX export → C++ ROS2 node (ONNX Runtime) → MuJoCo bridge → H1 in MuJoCo

## Environment

- Ubuntu 24.04 (WSL2), ROS2 Jazzy
