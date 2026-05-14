import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pico_bridge')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyACM0',
        description='Serial device for the Pico USB CDC')
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='115200',
        description='Serial baud (USB CDC ignores this, but kept for convention)')

    # Kill any stragglers from a previous launch that didn't shut down cleanly.
    # `;` (not `&&`) so a non-match (pkill returncode 1) doesn't abort the chain.
    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             "pkill -f 'lib/pico_bridge/serial_parser_node' ; "
             "pkill -f 'lib/pico_bridge/imu_node' ; "
             "pkill -f 'lib/pico_bridge/gps_node' ; "
             "pkill -f 'lib/pico_bridge/gps_to_odom_node' ; "
             "pkill -f 'lib/robot_localization/ekf_node' ; "
             "pkill -f 'lib/tf2_ros/static_transform_publisher' ; "
             "sleep 0.3 ; exit 0"],
        name='pico_bridge_cleanup',
        output='screen',
    )

    serial_parser = Node(
        package='pico_bridge',
        executable='serial_parser_node',
        name='serial_parser',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
        }],
    )

    imu_node = Node(
        package='pico_bridge',
        executable='imu_node',
        name='imu_node',
        output='screen',
    )

    gps_node = Node(
        package='pico_bridge',
        executable='gps_node',
        name='gps_node',
        output='screen',
    )

    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
    )

    tf_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_gps',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'gps_link'],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    gps_to_odom_node = Node(
        package='pico_bridge',
        executable='gps_to_odom_node',
        name='gps_to_odom',
        output='screen',
    )

    start_nodes_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                serial_parser,
                imu_node,
                gps_node,
                tf_base_to_imu,
                tf_base_to_gps,
                gps_to_odom_node,
                ekf_node,
            ],
        )
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        cleanup,
        start_nodes_after_cleanup,
    ])
