# dataset:=<name> picks configs/<name>/config.yaml out of the installed package.
# Pass config_filepath:= instead to point somewhere else.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("vio_node"), "configs", LaunchConfiguration("dataset"), "config.yaml"]
    )

    params = {
        "imu_topic": ("/imu/data", None),
        "image_topic": ("/camera/image_raw", None),
        "odom_topic": ("/vio/odom", None),
        "pose_topic": ("/vio/pose", None),
        "path_topic": ("/vio/path", None),
        "divergence_topic": ("/vio/divergence", None),
        "frame_id": ("odom", None),
        "body_frame_id": ("imu", None),
        "output_rate_hz": ("10.0", float),
        "imu_hold_max_s": ("1.0", float),
        "image_queue_max": ("30", int),
        "divergence_timeout_s": ("2.0", float),
        "divergence_pos_std_m": ("100.0", float),
        "path_max_poses": ("5000", int),
        "out_csv": ("", None),
        "use_sim_time": ("false", bool),
    }

    args = [
        DeclareLaunchArgument("dataset", default_value="AMtown03"),
        DeclareLaunchArgument("config_filepath", default_value=default_config),
        DeclareLaunchArgument("qos_profile", default_value="reliable",
                              choices=["reliable", "best_effort"]),
    ]
    args += [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in params.items()]

    # numeric parameters must carry their type: `output_rate_hz:=10` would otherwise
    # reach the node as an int and be rejected
    values = {
        k: (LaunchConfiguration(k) if t is None else ParameterValue(LaunchConfiguration(k), value_type=t))
        for k, (_, t) in params.items()
    }
    values["config_filepath"] = LaunchConfiguration("config_filepath")
    values["qos_profile"] = LaunchConfiguration("qos_profile")

    node = Node(
        package="vio_node",
        executable="vio_node",
        name="vio_node",
        output="screen",
        parameters=[values],
    )

    return LaunchDescription(args + [node])
