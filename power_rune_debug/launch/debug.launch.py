from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('power_rune_debug')
    
    # Config file path
    config_file = LaunchConfiguration('config_file')
    
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'armor_debug.yaml'),
        description='Path to the config file'
    )
    
    # Camera node (uncomment if needed)
    camera_node = Node(
        package='galaxy_camera',
        executable='galaxy_camera_node',
        name='galaxy_camera_node',
        output='screen'
    )
    
    # Debug node
    debug_node = Node(
        package='power_rune_debug',
        executable='power_rune_debug_node',
        name='power_rune_debug_node',
        output='screen',
        parameters=[{'config_file': config_file}]
    )
    
    # Debug GUI
    gui_node = Node(
        package='power_rune_debug',
        executable='power_rune_debug_gui',
        name='power_rune_debug_gui',
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        camera_node,
        debug_node,
        gui_node
    ])