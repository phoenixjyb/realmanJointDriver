from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('realman_arm_driver')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'arm_config.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN interface name'
    )
    
    # Create node
    realman_arm_node = Node(
        package='realman_arm_driver',
        executable='realman_arm_node',
        name='realman_arm_driver',
        parameters=[
            LaunchConfiguration('config_file'),
            {'can_interface': LaunchConfiguration('can_interface')}
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        can_interface_arg,
        realman_arm_node,
    ])
