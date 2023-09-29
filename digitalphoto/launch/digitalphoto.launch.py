from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(
            get_package_share_directory("digitalphoto"), "param", "digitalphoto.yaml"
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_dir",
                default_value=param_dir,
            ),
            Node(
                package="digitalphoto",
                executable="photoapp",
                parameters=[param_dir],
                output="screen",
            ),
            Node(
                package="digitalphoto",
                executable="googlephotodl",
                parameters=[param_dir],
                output="screen",
            ),
        ]
    )
