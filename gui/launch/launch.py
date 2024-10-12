import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui',  # Replace with your package name
            executable='gui',
            name='gui_node',
            output='screen',
        ),
        Node(
            package='gui',  # Replace with your package name
            executable='detection_visualizer',
            name='detection_visualizer_node',
            output='screen',
        ),
    ])
