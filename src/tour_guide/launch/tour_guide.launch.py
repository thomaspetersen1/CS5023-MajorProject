import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tour_guide")
    nav2_share = get_package_share_directory("nav2_bringup")
    rosbridge_share = get_package_share_directory("rosbridge_server")

    landmarks_file = os.path.join(pkg_share, "config", "landmarks.yaml")

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": os.path.join(pkg_share, "maps", "map1.yaml"),
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "rviz_launch.py")
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
    return LaunchDescription(
        [
            nav2_bringup,
            rviz,
            rosbridge,
            landmark_publisher,
            route_planner,
            tour_executor,
        ]
    )
