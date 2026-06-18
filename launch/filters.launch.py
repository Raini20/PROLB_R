from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Single-command launch for the full probabilistic robot lab setup.

    Starts everything in the correct order:
        0 s  — Gazebo + Nav2 + AMCL (no separate RViz)
        5 s  — KF, EKF, RViz, DataLogger
       10 s  — AutoNav: sets initial pose and drives fixed waypoints

    Usage:
        ros2 launch probabilistic_robot_lab filters.launch.py

    Optional flags:
        ros2 launch probabilistic_robot_lab filters.launch.py pf:=true
    """

    nav2_share = get_package_share_directory('nav2_bringup')

    return LaunchDescription([

        # ------------------------------------------------------------------
        # Launch arguments
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            'kf', default_value='true',
            description='Launch Kalman Filter node'),
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='Launch Extended Kalman Filter node'),
        DeclareLaunchArgument(
            'pf', default_value='false',
            description='Launch Particle Filter node (enable once implemented)'),

        # ------------------------------------------------------------------
        # 1.  Simulation — Gazebo + Nav2 + AMCL  (no Nav2 RViz)
        # ------------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'tb3_simulation_launch.py')
            ),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),

        # ------------------------------------------------------------------
        # 2.  Filter nodes + RViz + DataLogger  (delayed 5 s)
        # ------------------------------------------------------------------
        TimerAction(period=5.0, actions=[

            # KF node
            Node(
                package='probabilistic_robot_lab',
                executable='kf_node',
                name='kf_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('kf')),
            ),

            # EKF node
            Node(
                package='probabilistic_robot_lab',
                executable='ekf_node',
                name='ekf_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('ekf')),
            ),

            # PF node  (enable with pf:=true once pf_node is built)
            Node(
                package='probabilistic_robot_lab',
                executable='pf_node',
                name='pf_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('pf')),
            ),

            # RViz with our config
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('probabilistic_robot_lab'),
                    'rviz', 'filters.rviz'
                ])],
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),

            # Data logger — writes CSV to ~/prob_ros_ws/logs/
            Node(
                package='probabilistic_robot_lab',
                executable='data_logger',
                name='data_logger',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]),

        # ------------------------------------------------------------------
        # 3.  AutoNav — sets initial pose, then drives fixed waypoints
        # ------------------------------------------------------------------
        TimerAction(period=10.0, actions=[
            Node(
                package='probabilistic_robot_lab',
                executable='auto_nav',
                name='auto_nav',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]),
    ])
