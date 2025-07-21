# ros2 run realsense_node follower.py
# ros2 run realsense_node controller.py 


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_node',
            executable='follower.py',
            name='follower_node',
            output='screen',
        ),
        Node(
            package='realsense_node',
            executable='controller.py',
            name='controller_node',
            output='screen',
        )
    ])
