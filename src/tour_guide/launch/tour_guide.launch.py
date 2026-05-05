import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tour_guide")
    nav_share = get_package_share_directory("turtlebot4_navigation")
    viz_share = get_package_share_directory("turtlebot4_viz")
    rosbridge_share = get_package_share_directory("rosbridge_server")

    landmarks_file = os.path.join(pkg_share, "config", "landmarks.yaml")
    default_map = os.path.join(pkg_share, "maps", "map1.yaml")

    map_file = LaunchConfiguration("map")
    tour_startup_delay = LaunchConfiguration("tour_startup_delay")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "map": map_file,
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "nav2.launch.py")
        ),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(viz_share, "launch", "view_robot.launch.py")
        ),
    )
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                rosbridge_share, "launch", "rosbridge_websocket_launch.xml"
            )
        ),
    )
    landmark_publisher = Node(
        package="tour_guide",
        executable="landmark_publisher",
        name="landmark_publisher",
        output="screen",
        parameters=[{"landmarks_file": landmarks_file}],
    )
    route_planner = Node(
        package="tour_guide",
        executable="route_planner",
        name="route_planner",
        output="screen",
        parameters=[{"landmarks_file": landmarks_file}],
    )
    tour_executor = Node(
        package="tour_guide",
        executable="tour_executor",
        name="tour_executor",
        output="screen",
        parameters=[{"landmarks_file": landmarks_file}],
    )
    delayed_tour_nodes = TimerAction(
        period=tour_startup_delay,
        actions=[rosbridge, landmark_publisher, route_planner, tour_executor],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to the map yaml file to load.",
            ),
            DeclareLaunchArgument(
                "tour_startup_delay",
                default_value="10.0",
                description="Seconds to wait before starting tour-specific nodes.",
            ),
            localization,
            nav2,
            rviz,
            delayed_tour_nodes,
        ]
    )
