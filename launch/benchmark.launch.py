from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Start the UPSTREAM simulator separately. Do not assume the raw LiDAR topic
# exists until checked via `ros2 topic list -t`.
def generate_launch_description():
    args = [
        DeclareLaunchArgument('source_topic', default_value='/head/points'),
        DeclareLaunchArgument('backend', default_value='cpu'),
        DeclareLaunchArgument('relay', default_value='true'),
        DeclareLaunchArgument('map_frame', default_value=''),
        DeclareLaunchArgument('csv_path', default_value='/tmp/g1_bench_cpu.csv'),
        DeclareLaunchArgument('verify', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('voxel_size', default_value='0.10'),
        DeclareLaunchArgument('map_voxel_size', default_value='0.10'),
        DeclareLaunchArgument('max_map_voxels', default_value='300000'),
    ]
    relay = Node(
        package='g1_cuda_lidar_benchmark', executable='sensor_relay.py',
        output='screen', condition=IfCondition(LaunchConfiguration('relay')),
        parameters=[{'source_topic': LaunchConfiguration('source_topic'),
                     'output_topic': '/benchmark/raw_points',
                     'use_sim_time': LaunchConfiguration('use_sim_time')}])
    mapper = Node(
        package='g1_cuda_lidar_benchmark', executable='lidar_mapper',
        output='screen', parameters=[{
            'backend': LaunchConfiguration('backend'),
            'input_topic': '/benchmark/raw_points',
            'map_frame': LaunchConfiguration('map_frame'),
            'csv_path': LaunchConfiguration('csv_path'),
            'verify_against_cpu': LaunchConfiguration('verify'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'map_voxel_size': LaunchConfiguration('map_voxel_size'),
            'max_map_voxels': LaunchConfiguration('max_map_voxels'),
        }])
    return LaunchDescription(args+[relay,mapper])
