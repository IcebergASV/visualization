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
        Node(
            package='gui',
            executable='params_window',
            name='param_node',
            output='screen',
        ),
<<<<<<< HEAD
    ])
=======
    ])
>>>>>>> ab30b42f17c56075fef144c9f909c0aa9ee7fb96
