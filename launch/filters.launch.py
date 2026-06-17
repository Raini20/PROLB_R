from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Launch all probabilistic filter nodes.

    By default only the KF node is enabled.
    Enable others once implemented:
        ros2 launch probabilistic_robot_lab filters.launch.py ekf:=true pf:=true
    """
    return LaunchDescription([

        # ------------------------------------------------------------------
        # Launch arguments — set to true/false to enable/disable each filter
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            'kf', default_value='true',
            description='Launch Kalman Filter node'),
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='Launch Extended Kalman Filter node'),
        DeclareLaunchArgument(
            'pf', default_value='false',
            description='Launch Particle Filter node'),

        # ------------------------------------------------------------------
        # KF node
        # ------------------------------------------------------------------
        Node(
            package='probabilistic_robot_lab',
            executable='kf_node',
            name='kf_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('kf')),
        ),

        # ------------------------------------------------------------------
        # EKF node  (enable with ekf:=true once ekf_node is built)
        # ------------------------------------------------------------------
        Node(
            package='probabilistic_robot_lab',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('ekf')),
        ),

        # ------------------------------------------------------------------
        # PF node  (enable with pf:=true once pf_node is built)
        # ------------------------------------------------------------------
        Node(
            package='probabilistic_robot_lab',
            executable='pf_node',
            name='pf_node',
            output='screen',
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
        ),
    ])
