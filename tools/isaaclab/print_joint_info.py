"""Print the H1 joint order and per-joint settings as Isaac Lab resolves them."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="Isaac-Velocity-Flat-H1-v0")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym

import isaaclab_tasks  # noqa: F401  (importing registers the tasks)
from isaaclab_tasks.utils import parse_env_cfg

env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
env = gym.make(args.task, cfg=env_cfg)
robot = env.unwrapped.scene["robot"]

obs_dim = env.unwrapped.observation_manager.group_obs_dim["policy"]
act_dim = env.unwrapped.action_manager.total_action_dim
print(f"\n=== {args.task}: observation dim {obs_dim}, action dim {act_dim} ===")

print(f"{'idx':>3}  {'joint':<26} {'default':>8} {'kp':>7} {'kd':>6}")
for i, name in enumerate(robot.joint_names):
    default = robot.data.default_joint_pos[0, i].item()
    kp = robot.data.joint_stiffness[0, i].item()
    kd = robot.data.joint_damping[0, i].item()
    print(f"{i:>3}  {name:<26} {default:>8.3f} {kp:>7.1f} {kd:>6.1f}")


env.close()
simulation_app.close()