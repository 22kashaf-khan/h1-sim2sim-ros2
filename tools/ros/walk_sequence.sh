#!/usr/bin/env bash
# Publishes a fixed /cmd_vel sequence for repeatable walking measurements.
# Requires a sourced ROS2 environment.

phase() {
  local seconds=$1
  local twist=$2
  echo "$(date +%T)  ${seconds} s  ${twist}"
  timeout "$seconds" ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "$twist" > /dev/null
}

phase 5 "{}"
phase 10 "{linear: {x: 0.5}}"
phase 10 "{linear: {x: 1.0}}"
phase 8 "{angular: {z: 0.5}}"
phase 5 "{}"
echo "sequence done"