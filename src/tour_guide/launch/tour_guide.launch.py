import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tour_guide")
    nav2_share = get_package_share_directory("nav2_bringup")

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": os.path.join(pkg_share, "maps", "map.yaml"),
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "rviz_launch.py")
        ),
    )

    tour_executor = Node(
        package="tour_guide",
        executable="tour_executor",
        name="tour_executor",
        output="screen",
        parameters=[
            {
                "landmarks_file": os.path.join(pkg_share, "config", "landmarks.yaml"),
            }
        ],
    )

    # route_planner = Node(
    #     package='tour_guide',
    #     executable='route_planner',
    #     name='route_planner',
    #     output='screen',
    # )

    # landmark_publisher = Node(
    #     package='tour_guide',
    #     executable='landmark_publisher',
    #     name='landmark_publisher',
    #     output='screen',
    # )

    return LaunchDescription([nav2_bringup, rviz, tour_executor])
