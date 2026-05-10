from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get the package share directory
    pkg_share = get_package_share_directory('calian_gnss_ros2')
    
    # Path to parameter files
    config_file = os.path.join(pkg_share, 'params', 'config.yaml')
    logs_file = os.path.join(pkg_share, 'params', 'logs.yaml')
    # pointperfect_file = os.path.join(pkg_share, 'params', 'pointperfect.yaml')
    # ntrip_file = os.path.join(pkg_share, 'params', 'ntrip.yaml')

    return LaunchDescription([
        
        # RTK Corrections Listener
        Node(
            package='calian_gnss_ros2',
            executable='rtk_correction_listener',
            name='rtk_corrections_listener',
            namespace='calian_gnss',
            output='screen',
            emulate_tty=True
        ),

        # Base Node
        Node(
            package='calian_gnss_ros2',
            executable='calian_gnss_gps',
            name='base',
            namespace='calian_gnss',
            output='screen',
            arguments=['Heading_Base'],
            parameters=[
                config_file,
                logs_file,
                {'emulate_tty': True}
            ],
            remappings=[
                ('corrections', 'rtk_corrections')
            ],
            emulate_tty=True
        ),

        # Uncomment for PointPerfect Node
        # Node(
        #     package='calian_gnss_ros2',
        #     executable='pointperfect_module.py',
        #     name='pointperfect',
        #     namespace='calian_gnss',
        #     output='screen',
        #     parameters=[
        #         pointperfect_file,
        #         logs_file
        #     ],
        #     emulate_tty=True
        # ),

        # Uncomment for NTRIP Client
        # Node(
        #     package='calian_gnss_ros2',
        #     executable='ntrip_module.py',
        #     name='ntrip',
        #     namespace='calian_gnss',
        #     output='screen',
        #     parameters=[
        #         ntrip_file,
        #         logs_file
        #     ],
        #     emulate_tty=True
        # ),

        # Rover Node
        Node(
            package='calian_gnss_ros2',
            executable='calian_gnss_gps',
            name='rover',
            namespace='calian_gnss',
            output='screen',
            arguments=['Rover'],
            parameters=[
                config_file,
                logs_file,
                {'emulate_tty': True}
            ],
            emulate_tty=True
        ),

        # GPS Visualizer Node
        Node(
            package='calian_gnss_ros2',
            executable='calian_gnss_gps_visualizer',
            name='gps_visualizer',
            namespace='calian_gnss',
            output='screen',
            parameters=[
                {'port': 8080},
                {'emulate_tty': False}
            ],
            emulate_tty=False
        ),
    ])
