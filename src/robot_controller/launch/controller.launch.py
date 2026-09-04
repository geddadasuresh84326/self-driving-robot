from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition

def generate_launch_description():
   
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.0335"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.18"
    )
    use_diff_drive_controller_arg = DeclareLaunchArgument(
        "use_diff_drive_controller",
        default_value="True"
    )
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_diff_drive_controller = LaunchConfiguration("use_diff_drive_controller")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager"
        ])
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot_diff_drive_controller",
            "--controller-manager",
            "controller_manager"
        ],
        condition=IfCondition(use_diff_drive_controller)

    )
    simple_controller = GroupAction(
        condition=UnlessCondition(use_diff_drive_controller),
        actions=[
            Node( 
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "controller_manager"
                ],
            ),
            Node(
                package="robot_controller",
                executable="simple_controller",
                parameters=[{"wheel_radius": wheel_radius,
                            "wheel_separation": wheel_separation}],
            )
        ]
    )
    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        use_diff_drive_controller_arg,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        simple_controller
    ])