"""Summarize benchmark logs: velocity tracking per command phase, drift and falls."""

import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np

SETTLE_S = 2.0      # ignored at the start of each phase, while the robot adjusts
MIN_PHASE_S = 4.0   # shorter phases (e.g. gaps between commands) are skipped
FALL_HEIGHT_M = 0.5


def phases(log):
    """Yield (command, rows) for each stretch with a constant command."""
    cmd = np.stack([log["cmd_vx"], log["cmd_vy"], log["cmd_wz"]], axis=1)
    changes = np.nonzero(np.any(np.diff(cmd, axis=0) != 0, axis=1))[0] + 1
    starts = np.concatenate([[0], changes])
    ends = np.concatenate([changes, [len(log)]])
    for start, end in zip(starts, ends):
        t = log["time"][start:end]
        if t[-1] - t[0] < MIN_PHASE_S:
            continue
        yield tuple(cmd[start]), log[start:end][t >= t[0] + SETTLE_S]


def describe(cmd):
    names = ("vx", "vy", "wz")
    parts = [f"{name} {value:g}" for name, value in zip(names, cmd) if value != 0]
    return ", ".join(parts) if parts else "zero"


def fmt(values):
    values = np.asarray(values)
    return f"{values.mean():+.3f} ± {values.std():.3f}"


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("log_dir", type=Path, help="folder with run_<setting>_<n>.csv files")
args = parser.parse_args()

results = defaultdict(lambda: defaultdict(list))
falls = defaultdict(list)
for path in sorted(args.log_dir.glob("*.csv")):
    setting = path.stem.rsplit("_", 1)[0]
    log = np.genfromtxt(path, delimiter=",", names=True)
    falls[setting].append(bool(log["z"].min() < FALL_HEIGHT_M))

    per_command = defaultdict(list)
    for cmd, rows in phases(log):
        per_command[cmd].append(rows)
    for cmd, chunks in per_command.items():
        rows = np.concatenate(chunks)
        results[setting][cmd].append(
            (rows["vx_body"].mean(), rows["vy_body"].mean(), rows["wz_body"].mean()))

print("| Setting | Command | vx (m/s) | vy (m/s) | wz (rad/s) | Runs |")
print("|---|---|---|---|---|---|")
for setting in sorted(results):
    for cmd in sorted(results[setting]):
        runs = np.array(results[setting][cmd])
        print(f"| {setting} | {describe(cmd)} | {fmt(runs[:, 0])} | {fmt(runs[:, 1])} "
              f"| {fmt(runs[:, 2])} | {len(runs)} |")
print()
for setting in sorted(falls):
    print(f"{setting}: {sum(falls[setting])} falls in {len(falls[setting])} runs")