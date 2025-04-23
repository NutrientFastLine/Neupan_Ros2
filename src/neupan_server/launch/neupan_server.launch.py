from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    neupan_server_dir = get_package_share_directory('neupan_server')
    config_file_dir = os.path.join(neupan_server_dir, 'config', 'neupan_controller.yaml')
    weight_file_dir = os.path.join(neupan_server_dir, 'weight', 'model_5000.pth')
    rviz_file_dir = os.path.join(neupan_server_dir, 'rviz', 'display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config_file_dir),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('laser_frame', default_value='laser_link'),
        DeclareLaunchArgument('dune_checkpoint', default_value=weight_file_dir),

        Node(
            package='neupan_server',
            executable='neupan_server_node',
            name='neupan_server_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'laser_frame': LaunchConfiguration('laser_frame'),
                'scan_range': [0.0, 5.0],                      
                'scan_angle_range': [-3.14, 3.14],             
                'marker_size': 0.05,
                'marker_z': 0.3,
                'scan_downsample': 6,
                'dune_checkpoint': LaunchConfiguration('dune_checkpoint'),
            }],
            # remappings=[
            #     ('/neupan_cmd_vel', '/cmd_vel'),
            # ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file_dir]
        )
    ])
