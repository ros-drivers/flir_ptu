from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('flir_ptu_description'), 'urdf', 'example.urdf.xacro'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('flir_ptu_viz'), 'rviz', 'urdf.rviz'
    ])

    model_arg = DeclareLaunchArgument(
        'model', default_value='ptu5',
        description='PTU model to visualize (ptu5 or d46).',
    )
    publish_cmd_arg = DeclareLaunchArgument(
        'publish_cmd', default_value='false',
        description='Also publish marker values on the driver cmd topic.',
    )
    model = LaunchConfiguration('model')
    publish_cmd = LaunchConfiguration('publish_cmd')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' model:=', model]),
        value_type=str,
    )

    return LaunchDescription([
        model_arg,
        publish_cmd_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='flir_ptu_viz',
            executable='interactive_ptu.py',
            name='interactive_ptu',
            parameters=[{'publish_cmd': publish_cmd}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
