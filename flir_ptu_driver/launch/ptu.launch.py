from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('flir_ptu_driver')

    config_file = PathJoinSubstitution([pkg_share, 'config', 'ptu.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Path to the YAML parameter file for the PTU driver'),

        DeclareLaunchArgument(
            'namespace',
            default_value='ptu',
            description='Namespace for the PTU driver node'),

        Node(
            package='flir_ptu_driver',
            executable='ptu_node',
            name='ptu_driver',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('state', '/joint_states')],
            output='screen',
            respawn=True,
        ),
    ])
