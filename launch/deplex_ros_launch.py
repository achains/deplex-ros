from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="deplex_ros",
            executable="image_publisher",
            name="image_publisher_launch",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"image_dir": str(Path.cwd() / "data" / "TUM_fr3_long_val")}
            ]
        ),
        Node(
            package="deplex_ros",
            executable="image_listener",
            name="image_listener_launch",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"config_path": str(Path.cwd() / "data/config" / "TUM_fr3_long_val.ini"),
                 "intrinsics_path": str(Path.cwd() / "data/config" / "TUM_fr3_long_val.K")}
            ]
        ),
    ])
