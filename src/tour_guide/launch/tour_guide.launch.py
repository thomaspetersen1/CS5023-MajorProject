import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
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
    # rosbridge_share = get_package_share_directory("rosbridge_server")

    landmarks_file = os.path.join(pkg_share, "config", "landmarks.yaml")
    map_file = os.path.join(pkg_share, "maps", "map1.yaml")
    localization_params_file = os.path.join(
        pkg_share, "config", "localization.yaml"
    )
    nav2_params_file = os.path.join(nav_share, "config", "nav2.yaml")
    start_tour_nodes = LaunchConfiguration("start_tour_nodes")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "params": localization_params_file,
            "params_file": localization_params_file,
            "use_sim_time": "false",
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": "false",
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(viz_share, "launch", "view_robot.launch.py")
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )
    # rosbridge = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(
    #             rosbridge_share, "launch", "rosbridge_websocket_launch.xml"
    #         )
    #     ),
    # )
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
        period=30.0,
        actions=[landmark_publisher, route_planner, tour_executor],
        condition=IfCondition(start_tour_nodes),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_tour_nodes",
                default_value="false",
                description="Start tour nodes after localization/Nav2 startup.",
            ),
            localization,
            nav2,
            rviz,
            delayed_tour_nodes,
        ]
    )
