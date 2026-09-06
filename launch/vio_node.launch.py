# vio_node launch.
#
#   ros2 launch vio_node vio_node.launch.py
#   ros2 launch vio_node vio_node.launch.py config_filepath:=/abs/path/config.yaml cam_topic:=/cam0
#
# config_filepath defaults to the AMtown03 config shipped with the package.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("vio_node"), "configs", "AMtown03", "config.yaml"
    )

    # name -> (default, type). type=None means plain string.
    params = {
        "config_filepath": (default_config, None),
        "imu_topic": ("/imu/data", None),
        "cam_topic": ("/camera/image_raw", None),
        "odom_topic": ("/vio/odom", None),
        "pose_topic": ("/vio/pose", None),
        "path_topic": ("/vio/path", None),
        "divergence_topic": ("/vio/divergence", None),
        "frame_id": ("odom", None),
        "body_frame_id": ("imu", None),
        "output_rate_hz": ("10.0", float),
        "imu_hold_max_s": ("1.0", float),
        "divergence_timeout_s": ("2.0", float),
        "divergence_pos_std_m": ("100.0", float),
        "path_max_poses": ("2000", int),
        "out_csv": ("", None),
        "use_sim_time": ("false", bool),
    }

    args = [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in params.items()]
    # the node accepts exactly these two; `choices` is checked by launch only, so the node
    # validates the value again for the `ros2 run` path.
    args.append(
        DeclareLaunchArgument(
            "qos_profile", default_value="reliable", choices=["reliable", "best_effort"]
        )
    )

    # Numeric parameters must carry their type: a bare LaunchConfiguration passes the text,
    # and `output_rate_hz:=10` would then reach the node as an int and be rejected.
    values = {
        k: (LaunchConfiguration(k) if t is None else ParameterValue(LaunchConfiguration(k), value_type=t))
        for k, (_, t) in params.items()
    }
    values["qos_profile"] = LaunchConfiguration("qos_profile")

    node = Node(
        package="vio_node",
        executable="vio_node",
        name="vio_node",
        output="screen",
        parameters=[values],
    )

    return LaunchDescription(args + [node])
