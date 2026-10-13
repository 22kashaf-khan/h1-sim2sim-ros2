#!/usr/bin/env bash
# One benchmark run: start the simulation with logging, play the walking sequence, stop.
# Usage: run_benchmark.sh <isaac|model> <log_file.csv>

# Job control: the launch gets its own process group, like in an interactive terminal,
# so it can be stopped as a whole (launch + both nodes), exactly like Ctrl+C.
set -m

limits=$1
log=$2
if [ -z "$limits" ] || [ -z "$log" ]; then
  echo "usage: $0 <isaac|model> <log_file.csv>"
  exit 1
fi

if pgrep -f "mujoco_bridge|policy_node" > /dev/null; then
  echo "a simulation is still running; stop it before starting a benchmark run"
  exit 1
fi

ros2 launch h1_bringup sim2sim.launch.py torque_limits:="$limits" log_file:="$log" \
  timing_log:="${log%.csv}_timing.csv" \
  > "${log%.csv}_launch.txt" 2>&1 &
launch_pid=$!

sleep 5
"$(dirname "$0")/walk_sequence.sh"

kill -INT -- -"$launch_pid"
wait "$launch_pid"
sleep 1

if pgrep -f "mujoco_bridge|policy_node" > /dev/null; then
  echo "WARNING: simulation processes are still running after the run"
  exit 1
fi
echo "run finished: $log"