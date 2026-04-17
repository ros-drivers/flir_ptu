from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
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
    driver_launch = PathJoinSubstitution([
        FindPackageShare('flir_ptu_driver'), 'launch', 'ptu.launch.py'
    ])

    model_arg = DeclareLaunchArgument(
        'model', default_value='ptu5',
        description='PTU model being controlled (ptu5 or d46).',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='ptu',
        description='Namespace used by the PTU driver.',
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('flir_ptu_driver'), 'config', 'ptu.yaml'
        ]),
        description='YAML parameter file passed to the PTU driver.',
    )

    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' model:=', model]),
        value_type=str,
    )

    return LaunchDescription([
        model_arg,
        namespace_arg,
        params_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([driver_launch]),
            launch_arguments={
                'namespace': namespace,
                'params_file': LaunchConfiguration('params_file'),
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='flir_ptu_viz',
            executable='interactive_ptu.py',
            name='interactive_ptu',
            parameters=[{
                'publish_cmd': True,
                'publish_joint_states': False,
            }],
            remappings=[
                ('cmd', [TextSubstitution(text='/'), namespace, TextSubstitution(text='/cmd')]),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
