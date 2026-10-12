"""Start the MuJoCo bridge and the policy node for H1 sim-to-sim."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Isaac Lab training limits (env.yaml, effort_limit_sim): 100 Nm for the ankles, 300 Nm otherwise.
ISAAC_ANKLE_LIMIT = 100.0
ISAAC_OTHER_LIMIT = 300.0


def launch_setup(context):
    policy = LaunchConfiguration("policy").perform(context)
    torque_limits = LaunchConfiguration("torque_limits").perform(context)
    scene = LaunchConfiguration("scene").perform(context)

    policy_dir = os.path.join(get_package_share_directory("h1_policy"), "policies", policy)
    deploy_path = os.path.join(policy_dir, "deploy.yaml")

    bridge_params = {"scene": scene, "deploy_config": deploy_path}
    if torque_limits == "isaac":
        with open(deploy_path) as f:
            joint_names = yaml.safe_load(f)["joint_names"]
        bridge_params["torque_limits"] = [
            ISAAC_ANKLE_LIMIT if "ankle" in name else ISAAC_OTHER_LIMIT for name in joint_names
        ]

    return [
        Node(
            package="h1_mujoco_bridge",
            executable="mujoco_bridge",
            parameters=[bridge_params],
            output="screen",
        ),
        Node(
            package="h1_policy",
            executable="policy_node",
            parameters=[{"policy_dir": policy_dir}],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("policy", default_value="h1_flat_baseline"),
        DeclareLaunchArgument("torque_limits", default_value="isaac", choices=["isaac", "model"]),
        DeclareLaunchArgument(
            "scene",
            default_value=os.path.expanduser("~/libs/mujoco_menagerie/unitree_h1/scene.xml"),
        ),
        OpaqueFunction(function=launch_setup),
    ])