import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    gui_node = Node(
        package='gui',  # Replace with your package name
        executable='gui',
        name='gui_node',
        output='screen',
    )

    detection_visualizer_node = Node(
        package='gui',  
        executable='detection_visualizer',
        name='detection_visualizer_node',
        output='screen',
    )

    param_node = Node(
        package='gui',
        executable='params_window',
        name='param_node',
        output='screen',
    )

    return LaunchDescription([
        gui_node,
        detection_visualizer_node,
        param_node,

        # Ensure all nodes exit when any node stops
        RegisterEventHandler(
            OnProcessExit(
                target_action=gui_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=detection_visualizer_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=param_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
