from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='lakibeam1_driver',
            executable='lakibeam1_node',
            name='lakibeam1',
            output='screen',
            parameters=[
                {'port': 2368},
                {'frame_id': 'laser'},
            ]
        )
    ])
