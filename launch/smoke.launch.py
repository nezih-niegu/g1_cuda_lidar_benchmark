"""Pure ROS/DDS smoke test without MuJoCo; never use as simulated evidence."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='cpu'),
        DeclareLaunchArgument('points', default_value='16384'),
        Node(package='g1_cuda_lidar_benchmark', executable='synthetic_lidar.py',
             parameters=[{'points': LaunchConfiguration('points')}]),
        Node(package='g1_cuda_lidar_benchmark', executable='lidar_mapper',
             parameters=[{'backend': LaunchConfiguration('backend'),
                          'input_topic': '/benchmark/raw_points',
                          'csv_path': '/tmp/g1_smoke.csv',
                          'verify_against_cpu': True,
                          'use_sim_time': False}]),
    ])
