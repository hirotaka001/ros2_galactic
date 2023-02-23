from launch import LaunchDescription
import launch.substitutions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    params_file = LaunchConfiguration(
        'params',
        default=[ThisLaunchFileDir(), '/params.yaml'])
    
    return LaunchDescription([
        Node(
            package='hello_world',
            executable='talker_with_service_param',
            name='talker',
            output='screen',
            parameters=[params_file]),
        
        Node(
            package='hello_world',
            executable='listener',
            output='screen'),
    ])