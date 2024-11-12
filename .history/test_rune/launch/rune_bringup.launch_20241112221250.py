import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('test_rune'), 'launch'))

def generate_launch_description():

    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('test_rune'), 'config', 'launch_params.yaml')))

    node_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('test_rune'), 'config', 'node_params.yaml')))

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            #parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    galaxy_camera_node = get_camera_node('galaxy_camera', 'galaxy_camera::GalaxyCameraNode')
    
    power_rune_node = Node(
        package='power_rune',
        executable='power_rune_node',
        name='power_rune',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', 
                       '--log-level',
                       'power_rune_driver:='+launch_params['power_rune_log_level']],
    
    )

    return LaunchDescription([
        galaxy_camera_node,
        power_rune_node
    ])