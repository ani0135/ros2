import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch first instance of turtlesim_node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1',
            output='screen',
            parameters=[{'background_brightness': 200}],
        ),
        # Launch second instance of turtlesim_node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle2',
            output='screen',
            parameters=[{'background_brightness': 200}],
        ),
        # Launch a node to control both turtles (use a custom control node)
        Node(
            package='turtle_teleop',
            executable='teleop_turtle',  # Assuming this is the teleop executable
            name='teleop',
            output='screen',
            parameters=[{'turtle_name': 'turtle1'}],
        ),
        Node(
            package='turtle_teleop',
            executable='teleop_turtle',  # Adjust to control second turtle
            name='teleop_turtle2',
            output='screen',
            parameters=[{'turtle_name': 'turtle2'}],
        ),
    ])
