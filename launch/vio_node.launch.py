# vio_node launch.
#
#   ros2 launch vio_node vio_node.launch.py config_filepath:=/abs/path/config.yaml
#
# config_filepath is required. imu_topic / cam_topic default to the dataset names here
# (the node itself requires them). Everything else has defaults (see src/vio_node.cpp).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("config_filepath"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("cam_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("odom_topic", default_value="/vio/odom"),
        DeclareLaunchArgument("pose_topic", default_value="/vio/pose"),
        DeclareLaunchArgument("path_topic", default_value="/vio/path"),
        DeclareLaunchArgument("divergence_topic", default_value="/vio/divergence"),
        DeclareLaunchArgument("reliable_qos", default_value="true"),
        DeclareLaunchArgument("reorder_lag_s", default_value="0.05"),
        DeclareLaunchArgument("output_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("frame_id", default_value="global"),
        DeclareLaunchArgument("body_frame_id", default_value="imu"),
        DeclareLaunchArgument("divergence_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("divergence_pos_std_m", default_value="100.0"),
        DeclareLaunchArgument("path_max_poses", default_value="5000"),
        DeclareLaunchArgument("out_csv", default_value=""),
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
                "odom_topic": LaunchConfiguration("odom_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "path_topic": LaunchConfiguration("path_topic"),
                "divergence_topic": LaunchConfiguration("divergence_topic"),
                "reliable_qos": LaunchConfiguration("reliable_qos"),
                "reorder_lag_s": LaunchConfiguration("reorder_lag_s"),
                "output_rate_hz": LaunchConfiguration("output_rate_hz"),
                "frame_id": LaunchConfiguration("frame_id"),
                "body_frame_id": LaunchConfiguration("body_frame_id"),
                "divergence_timeout_s": LaunchConfiguration("divergence_timeout_s"),
                "divergence_pos_std_m": LaunchConfiguration("divergence_pos_std_m"),
                "path_max_poses": LaunchConfiguration("path_max_poses"),
                "out_csv": LaunchConfiguration("out_csv"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(args + [node])
