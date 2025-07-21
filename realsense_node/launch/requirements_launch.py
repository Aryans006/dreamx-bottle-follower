from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # mavros_launch = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(  # Use AnyLaunch to handle .launch (XML) format
    #         os.path.join(
    #             get_package_share_directory('mavros'),
    #             'launch/apm.launch'
    #         )
    #     ),
    #     launch_arguments={
    #         'fcu_url': '/dev/ttyACM0'
    #     }.items()
    # )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch/rs_launch.py'
            )
        ),
        launch_arguments={
            'depth_module.depth_profile': '1280x720x30',
            'pointcloud.enable': 'true'
        }.items()
    )

    waypoint_pub = TimerAction(
        period=5.0, 
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '/mavros/mission/reached',
                    'mavros_msgs/msg/WaypointReached',
                    '{wp_seq: 3}'
                ],
                shell=True
            )
        ]
    )

    return LaunchDescription([
        # mavros_launch,
        realsense_launch,
        waypoint_pub
    ])
