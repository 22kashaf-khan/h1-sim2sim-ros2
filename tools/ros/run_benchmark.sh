#!/usr/bin/env bash
# One benchmark run: start the simulation with logging, play the walking sequence, stop.
# Usage: run_benchmark.sh <isaac|model> <log_file.csv>

limits=$1
log=$2
if [ -z "$limits" ] || [ -z "$log" ]; then
  echo "usage: $0 <isaac|model> <log_file.csv>"
  exit 1
fi

ros2 launch h1_bringup sim2sim.launch.py torque_limits:="$limits" log_file:="$log" \
  > "${log%.csv}_launch.txt" 2>&1 &
launch_pid=$!

sleep 5
"$(dirname "$0")/walk_sequence.sh"

kill -TERM "$launch_pid"
wait "$launch_pid"
echo "run finished: $log"
