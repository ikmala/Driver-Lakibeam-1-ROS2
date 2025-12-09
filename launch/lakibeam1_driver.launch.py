from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Aktifkan log debug driver'
    )

    debug_param = ParameterValue(
        LaunchConfiguration('debug'),
        value_type=bool
    )

    decimation_arg = DeclareLaunchArgument(
        'publish_decimation',
        default_value='1',
        description='Kirim hanya setiap N-scan (throttling)'
    )

    decimation_param = ParameterValue(
        LaunchConfiguration('publish_decimation'),
        value_type=int
    )

    return LaunchDescription([
        debug_arg,
        decimation_arg,
        Node(
            package='lakibeam1_driver',
            executable='lakibeam1_node',
            name='lakibeam1',
            output='screen',
            parameters=[
                {'port': 2368},
                {'frame_id': 'laser'},
                {'debug': debug_param},
                {'publish_decimation': decimation_param},
            ]
        )
    ])
