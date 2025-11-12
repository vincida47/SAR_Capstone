from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    pkg = FindPackageShare('41068_ignition_bringup')

    # Start Cartographer + Nav2 first
    mapping_and_nav = IncludeLaunchDescription(
        PathJoinSubstitution([pkg, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Start explorer after 8 s (Nav2 + Cartographer settled)
    explorer = Node(
        package='41068_ignition_bringup',
        executable='auto_explore_waypoints.py',
        name='auto_explore_waypoints',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        mapping_and_nav,
        TimerAction(period=8.0, actions=[explorer])
    ])
