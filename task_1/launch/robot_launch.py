from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('task_1'), 'urdf', 'custom_robot.urdf')
    world_path = os.path.join(get_package_share_directory('task_1'), 'worlds', 'simple_world.world')

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        ),
        
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', '~/ros2_ws/src/task_1/rviz/config.rviz']
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_robot', '-file', urdf_path],
            output='screen'
        ),
        
        Node(
            package='task_1',
            executable='camera_publisher',
            output='screen'
        ),
        
        Node(
            package='task_1',
            executable='image_subscriber',
            output='screen'
        ),
    ])
