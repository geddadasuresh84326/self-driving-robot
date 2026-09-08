import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_firmware"),
                "launch",
                "hardware_interface.launch.py",
            )
        )
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_controller"),
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={"use_diff_drive_controller": "True"}.items(),
    )

    return LaunchDescription([hardware_interface, controller])
