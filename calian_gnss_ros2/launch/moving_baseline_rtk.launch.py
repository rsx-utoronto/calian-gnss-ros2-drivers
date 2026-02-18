import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("calian_gnss_ros2"), "params", "config.yaml"
    )

    corrections_config = os.path.join(
        get_package_share_directory("calian_gnss_ros2"), "params", "pointperfect.yaml"
    )

    ntrip_corrections_config = os.path.join(
        get_package_share_directory("calian_gnss_ros2"), "params", "ntrip.yaml"
    )

    logs_config = os.path.join(
        get_package_share_directory("calian_gnss_ros2"), "params", "logs.yaml"
    )

    return LaunchDescription(
        [
            # RTK Corrections Listener
            Node(
                package="calian_gnss_ros2",
                executable="rtk_correction_listener",
                name="rtk_corrections_listener",
                output="screen",
                namespace="calian_gnss",
            ),
            # Base Node
            Node(
                package="calian_gnss_ros2",
                executable="calian_gnss_gps",
                name="base",
                output="screen",
                emulate_tty=True,
                parameters=[config, logs_config],
                namespace="calian_gnss",
                remappings=[("corrections", "rtk_corrections")],
                arguments=["Heading_Base"],
            ),
            # Uncomment for PointPerfect Node
            # Node(
            #     package="calian_gnss_ros2",
            #     executable="pointperfect",
            #     name="pointperfect",
            #     output="screen",
            #     emulate_tty=True,
            #     parameters=[corrections_config, logs_config],
            #     namespace="calian_gnss",
            # ),
            # Uncomment for NTRIP Client
            # Node(
            #     package="calian_gnss_ros2",
            #     executable="ntrip_client",
            #     name="ntrip",
            #     output="screen",
            #     emulate_tty=True,
            #     parameters=[ntrip_corrections_config, logs_config],
            #     namespace="calian_gnss",
            # ),
            # Rover Node
            Node(
                package="calian_gnss_ros2",
                executable="calian_gnss_gps",
                name="rover",
                output="screen",
                emulate_tty=True,
                parameters=[config, logs_config],
                namespace="calian_gnss",
                arguments=["Rover"],
            ),
            # GPS Visualizer Node
            Node(
                package="calian_gnss_ros2",
                executable="calian_gnss_gps_visualizer",
                name="gps_visualizer",
                output="screen",
                emulate_tty=False,
                parameters=[{"port": 8080}],
                namespace="calian_gnss",
            ),
        ]
    )
