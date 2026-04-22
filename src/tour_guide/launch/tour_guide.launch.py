import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('tour_guide')

    # --- Launch args (overridable from the command line) -------------------
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'tour_map.yaml'),
        description='Path to the map YAML (map_server format)',
    )
    landmarks_arg = DeclareLaunchArgument(
        'landmarks_file',
        default_value=os.path.join(pkg_share, 'config', 'landmarks.yaml'),
        description='Path to the landmarks YAML',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo clock (true for simulation, false on the real bot)',
    )

    map_yaml = LaunchConfiguration('map')
    landmarks_file = LaunchConfiguration('landmarks_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Nav2 bringup (map_server + amcl + planner + controller + BT) ------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # --- Our three nodes ---------------------------------------------------
    common_params = [
        {'use_sim_time': use_sim_time},
        {'landmarks_file': landmarks_file},
    ]

    route_planner = Node(
        package='tour_guide',
        executable='route_planner',
        name='route_planner',
        output='screen',
        parameters=common_params,
    )

    tour_executor = Node(
        package='tour_guide',
        executable='tour_executor',
        name='tour_executor',
        output='screen',
        parameters=common_params,
    )

    landmark_publisher = Node(
        package='tour_guide',
        executable='landmark_publisher',
        name='landmark_publisher',
        output='screen',
        parameters=common_params,
    )

    return LaunchDescription([
        map_arg,
        landmarks_arg,
        use_sim_time_arg,
        nav2_bringup,
        route_planner,
        tour_executor,
        landmark_publisher,
    ])