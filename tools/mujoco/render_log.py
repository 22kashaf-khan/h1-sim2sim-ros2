"""Render a mujoco_bridge CSV log as an MP4 video, with a camera following the robot."""

import argparse
import csv
from pathlib import Path

import imageio.v2 as imageio
import mujoco

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("log", type=Path, help="CSV written by mujoco_bridge (log_file parameter)")
parser.add_argument(
    "--scene", type=Path,
    default=Path.home() / "libs/mujoco_menagerie/unitree_h1/scene.xml")
parser.add_argument("--output", type=Path, help="default: the log path with .mp4")
parser.add_argument("--start", type=float, default=0.0, help="seconds")
parser.add_argument("--end", type=float, default=None, help="seconds")
parser.add_argument("--fps", type=int, default=25)
args = parser.parse_args()

model = mujoco.MjModel.from_xml_path(str(args.scene))
data = mujoco.MjData(model)

with args.log.open() as f:
    rows = list(csv.DictReader(f))

base_columns = ["x", "y", "z", "qw", "qx", "qy", "qz"]
joint_columns = [name for name in rows[0] if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0]
joint_qpos = [model.joint(name).qposadr[0] for name in joint_columns]

log_rate = 1.0 / (float(rows[1]["time"]) - float(rows[0]["time"]))
every = max(1, round(log_rate / args.fps))

camera = mujoco.MjvCamera()
camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
camera.trackbodyid = model.body("pelvis").id
camera.distance = 3.5
camera.elevation = -15.0
camera.azimuth = 135.0

output = args.output or args.log.with_suffix(".mp4")
renderer = mujoco.Renderer(model, height=480, width=640)
frames = 0
with imageio.get_writer(output, fps=args.fps) as writer:
    for row in rows[::every]:
        t = float(row["time"])
        if t < args.start or (args.end is not None and t > args.end):
            continue
        data.qpos[:7] = [float(row[c]) for c in base_columns]
        for column, adr in zip(joint_columns, joint_qpos):
            data.qpos[adr] = float(row[column])
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        writer.append_data(renderer.render())
        frames += 1

print(f"wrote {output}: {frames} frames, {frames / args.fps:.1f} s")