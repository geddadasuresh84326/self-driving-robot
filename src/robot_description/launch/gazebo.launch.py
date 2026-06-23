from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_description_dir = get_package_share_directory("robot_description")
    # pkg_share_dir = get_package_share_directory("self_driving_pkg")
    # model_share_dir = os.path.join(pkg_share_dir,"models")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(robot_description_dir,"urdf","robot.urdf.xacro"),
        description="Absolute path to the urdf model"
    )
     # GAZEBO_MODEL_PATH 
    # os.environ['GAZEBO_MODEL_PATH'] = (
        # os.environ.get("GAZEBO_MODEL_PATH",'') + os.pathsep + model_share_dir )
    robot_description = ParameterValue(Command([
        "xacro ", LaunchConfiguration('model'),
        ]),
        value_type=str)
    
    # gazebo_world_arg = DeclareLaunchArgument(
    #     "world",
    #     default_value=os.path.join(get_package_share_directory("self_driving_pkg"),"worlds","line_aruco2.world"),
    #     description="custom world with models")

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(get_package_share_directory("robot_description"),"rviz","display.rviz"),
        description="rviz configuration"
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['joint_states']}]
    )
    # gazebo server
    gzserver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
    )
    # gazebo client
    gzclient_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # This node needs to get the robot_description parameter from the robot_state_publisher
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'self_driving_robot',
                   '-x', '-3.4',    # X position
                   '-y', '-4.6',    # Y position
                   '-z', '0.0',    # Z height
                   '-Y', '1.5708'     # Yaw (rotation around Z, in radians)
        ],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )
    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[os.path.join(
    #         get_package_share_directory("bumperbot_controller"),
    #         "config", "bumperbot_controllers.yaml"
    #     ), {'robot_description': robot_description}],
    #     output="screen",
    # )
    
    return LaunchDescription([
        rviz_config_arg,
        # gazebo_world_arg,
        model_arg,
        robot_state_publisher,
        gzserver_node,
        gzclient_node,
        spawn_entity_node,
        # controller_manager,
        joint_state_publisher,
        rviz_node,
        
    ])