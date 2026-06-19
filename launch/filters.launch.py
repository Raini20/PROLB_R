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

    Startup order:
        0 s  — Gazebo + Nav2 + AMCL (no Nav2 RViz)
        5 s  — KF, EKF, PF (if enabled), RViz, DataLogger
       10 s  — AutoNav: sets initial pose and drives fixed waypoints

    Usage:
        # Default: KF + EKF
        ros2 launch probabilistic_robot_lab filters.launch.py

        # Enable PF (standard, with resampling)
        ros2 launch probabilistic_robot_lab filters.launch.py pf:=true

        # Special Task: PF without resampling (particle degeneration)
        ros2 launch probabilistic_robot_lab filters.launch.py pf:=true resampling:=false

        # Noise experiments (vary Q/R for KF and EKF)
        ros2 launch probabilistic_robot_lab filters.launch.py \\
            sigma_process:=0.1 sigma_meas:=0.01
    """

    nav2_share = get_package_share_directory('nav2_bringup')

    return LaunchDescription([

        # ------------------------------------------------------------------
        # Launch arguments
        # ------------------------------------------------------------------
        DeclareLaunchArgument('kf',  default_value='true',
                              description='Launch Kalman Filter node'),
        DeclareLaunchArgument('ekf', default_value='true',
                              description='Launch Extended Kalman Filter node'),
        DeclareLaunchArgument('pf',  default_value='true',
                              description='Launch Particle Filter node'),

        # Noise experiment parameters (KF + EKF)
        DeclareLaunchArgument('sigma_process', default_value='0.01',
                              description='Process noise std dev (scales R)'),
        DeclareLaunchArgument('sigma_meas',    default_value='0.10',
                              description='Measurement noise std dev (scales Q)'),

        # PF parameters
        DeclareLaunchArgument('num_particles', default_value='500',
                              description='Number of PF particles'),
        DeclareLaunchArgument('resampling',    default_value='true',
                              description='Enable resampling in PF (false = Special Task)'),

        # ------------------------------------------------------------------
        # 1. Simulation — Gazebo + Nav2 (no Nav2 RViz)
        # ------------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'tb3_simulation_launch.py')
            ),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),

        # ------------------------------------------------------------------
        # 2. Filter nodes + RViz + DataLogger (delayed 5 s)
        # ------------------------------------------------------------------
        TimerAction(period=5.0, actions=[

            Node(
                package='probabilistic_robot_lab',
                executable='kf_node',
                name='kf_node',
                output='screen',
                parameters=[{
                    'use_sim_time':   True,
                    'sigma_process':  LaunchConfiguration('sigma_process'),
                    'sigma_meas':     LaunchConfiguration('sigma_meas'),
                }],
                condition=IfCondition(LaunchConfiguration('kf')),
            ),

            Node(
                package='probabilistic_robot_lab',
                executable='ekf_node',
                name='ekf_node',
                output='screen',
                parameters=[{
                    'use_sim_time':   True,
                    'sigma_process':  LaunchConfiguration('sigma_process'),
                    'sigma_meas':     LaunchConfiguration('sigma_meas'),
                }],
                condition=IfCondition(LaunchConfiguration('ekf')),
            ),

            Node(
                package='probabilistic_robot_lab',
                executable='pf_node',
                name='pf_node',
                output='screen',
                parameters=[{
                    'use_sim_time':   True,
                    'num_particles':  LaunchConfiguration('num_particles'),
                    'resampling':     LaunchConfiguration('resampling'),
                }],
                condition=IfCondition(LaunchConfiguration('pf')),
            ),

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

            Node(
                package='probabilistic_robot_lab',
                executable='data_logger',
                name='data_logger',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),

            Node(
                package='probabilistic_robot_lab',
                executable='landmark_viz',
                name='landmark_viz',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]),

        # ------------------------------------------------------------------
        # 3. AutoNav — sets initial pose and drives fixed waypoints
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
