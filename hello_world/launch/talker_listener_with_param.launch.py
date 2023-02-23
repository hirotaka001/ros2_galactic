from launch import LaunchDescription
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'decoration',
            default_value=TextSubstitution(text='def'),
            description='Message decoration string'),
        Node(
            name='talker',
            package='hello_world',
            executable='talker_with_service_param',
            output='screen',
            parameters=[{
                'decoration': LaunchConfiguration('decoration')
            }]),
        Node(
            package='hello_world',
            executable='listener',
            output='screen'),
    ])