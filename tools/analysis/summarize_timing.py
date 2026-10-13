"""Summarize policy_node timing logs: compute time, loop interval (jitter) and stalls."""

import argparse
from pathlib import Path

import numpy as np

PERIOD_MS = 20.0
LATE_MS = 1.5 * PERIOD_MS
STALL_MS = 100.0

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("timing_logs", type=Path, nargs="+", help="*_timing.csv files")
args = parser.parse_args()

for path in args.timing_logs:
    log = np.genfromtxt(path, delimiter=",", names=True)
    compute = log["compute_us"][log["published"] == 1]
    interval = log["interval_ms"][1:]
    elapsed = log["elapsed_s"][1:]

    print(f"== {path.name}: {len(log)} ticks, {log['elapsed_s'][-1]:.0f} s")
    print(f"compute (us):  median {np.median(compute):.0f}, "
          f"p99 {np.percentile(compute, 99):.0f}, max {compute.max():.0f}")
    print(f"budget used:   {100 * np.median(compute) / (1000 * PERIOD_MS):.2f} % of {PERIOD_MS:.0f} ms")
    print(f"interval (ms): median {np.median(interval):.2f}, "
          f"p1 {np.percentile(interval, 1):.2f}, p99 {np.percentile(interval, 99):.2f}, "
          f"max {interval.max():.1f}")
    print(f"late ticks:    {np.sum(interval > LATE_MS)} of {len(interval)} over {LATE_MS:.0f} ms")

    stalls = elapsed[interval > STALL_MS]
    if len(stalls) == 0:
        print("stalls:        none")
    else:
        print("stalls at (s): " + ", ".join(f"{t:.1f}" for t in stalls))
        if len(stalls) > 1:
            print("spacing (s):   " + ", ".join(f"{d:.1f}" for d in np.diff(stalls)))
    print()