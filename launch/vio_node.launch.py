# vio_node launch.
#
# Usage:
#   ros2 launch vio_node vio_node.launch.py config_filepath:=/abs/path/to/config.yaml
#
# The node requires config_filepath, imu_topic and cam_topic; the rest fall back
# to defaults inside src/vio_node.cpp.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("config_filepath"),   # required
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("cam_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("pose_topic", default_value="/vio/pose"),
        DeclareLaunchArgument("path_topic", default_value="/vio/path"),
        DeclareLaunchArgument("image_topic", default_value="/vio/tracks"),
        DeclareLaunchArgument("extrinsics_topic", default_value="/vio/extrinsics"),
        DeclareLaunchArgument("intrinsics_topic", default_value="/vio/intrinsics"),
        DeclareLaunchArgument("origin_topic", default_value="/vio/origin"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("bagfile", default_value="msceqf_record"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    node = Node(
        package="vio_node",
        executable="vio_node",
        name="vio_node",
        output="screen",
        parameters=[
            {
                "config_filepath": LaunchConfiguration("config_filepath"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "cam_topic": LaunchConfiguration("cam_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "path_topic": LaunchConfiguration("path_topic"),
                "image_topic": LaunchConfiguration("image_topic"),
                "extrinsics_topic": LaunchConfiguration("extrinsics_topic"),
                "intrinsics_topic": LaunchConfiguration("intrinsics_topic"),
                "origin_topic": LaunchConfiguration("origin_topic"),
                "record": LaunchConfiguration("record"),
                # the node reads this parameter as "outbag" (src/vio_node.cpp);
                # passing it as "bagfile" made record:=true abort with
                # "Recording enabled and output bagfile not defined".
                "outbag": LaunchConfiguration("bagfile"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(args + [node])
