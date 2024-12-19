from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description() -> LaunchDescription:
    '''
    This function is used to generate the launch file for hte imu_driver node.

    Args:
        None

    Returns:
        LaunchDescription: The launch description for the imu_driver node.

    Raises:
        None
    '''
    
    return LaunchDescription([
        DeclareLaunchArgument(
                name='port',
                default_value='/dev/ttyUSB0',
                description='The port where the IMU is connected.'
            ),
        Node(
                package='imu_driver',
                executable='talker',
                name='talker',
                output='screen',
                parameters=[{'port': LaunchConfiguration('port'),
                             'baudrate': 115200}]
            ),

    ])