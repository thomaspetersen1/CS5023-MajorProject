import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_share = get_package_share_directory('turtlebot4_navigation')
    viz_share = get_package_share_directory('turtlebot4_viz')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'slam.launch.py')
        ),
    )
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(viz_share, 'launch', 'view_robot.launch.py')
        ),
    )

    return LaunchDescription([slam_launch, viz_launch])