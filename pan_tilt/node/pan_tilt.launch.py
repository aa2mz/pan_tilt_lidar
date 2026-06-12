from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the Feetech USB controller board',
        ),
        DeclareLaunchArgument('baud_rate',      default_value='1000000'),
        DeclareLaunchArgument('pan_id',         default_value='1'),
        DeclareLaunchArgument('tilt_id',        default_value='2'),
        DeclareLaunchArgument('pan_min_deg',    default_value='-135.0'),
        DeclareLaunchArgument('pan_max_deg',    default_value='135.0'),
        DeclareLaunchArgument('tilt_min_deg',   default_value='-90.0'),
        DeclareLaunchArgument('tilt_max_deg',   default_value='45.0'),
        DeclareLaunchArgument('default_speed',  default_value='500'),
        DeclareLaunchArgument('feedback_hz',    default_value='20.0'),

        Node(
            package='pan_tilt',
            executable='pan_tilt_node.py',
            name='pan_tilt',
            output='screen',
            parameters=[{
                'serial_port':   LaunchConfiguration('serial_port'),
                'baud_rate':     LaunchConfiguration('baud_rate'),
                'pan_id':        LaunchConfiguration('pan_id'),
                'tilt_id':       LaunchConfiguration('tilt_id'),
                'pan_min_deg':   LaunchConfiguration('pan_min_deg'),
                'pan_max_deg':   LaunchConfiguration('pan_max_deg'),
                'tilt_min_deg':  LaunchConfiguration('tilt_min_deg'),
                'tilt_max_deg':  LaunchConfiguration('tilt_max_deg'),
                'default_speed': LaunchConfiguration('default_speed'),
                'feedback_hz':   LaunchConfiguration('feedback_hz'),
            }],
        ),
    ])
