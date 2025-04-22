import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_share_directory = FindPackageShare(package='neupan_server').find('neupan_server')
    config_file_path = os.path.join(package_share_directory, 'config', 'neupan_controller.yaml')
    dune_checkpoint = os.path.join(package_share_directory, 'pretrain_limo', 'model_5000.pth')
    
    map_frame = LaunchConfiguration('map_frame', default='map')
    base_frame = LaunchConfiguration('base_frame', default='base_link')
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser_link')
    marker_size = LaunchConfiguration('marker_size', default='0.05')
    marker_z = LaunchConfiguration('marker_z', default='0.3')
    scan_angle_range = LaunchConfiguration('scan_angle_range', default='-3.14 3.14')
    scan_downsample = LaunchConfiguration('scan_downsample', default='6')
    scan_range = LaunchConfiguration('usart_port', default='0.0 5.0')
    refresh_initial_path = LaunchConfiguration('refresh_initial_path', default='False')
    flip_angle = LaunchConfiguration('flip_angle', default='False')

    neupan_server_node = Node(
        package='neupan_server',
        executable='neupan_server_node',
        name='neupan_server_node',
        parameters=[
            {
                'config_file_path': config_file_path,
                'dune_checkpoint': dune_checkpoint,
                'map_frame': map_frame,
                'base_frame': base_frame,
                'lidar_frame' : lidar_frame,
                'marker_size': marker_size,
                'marker_z' : marker_z,
                'scan_angle_range' : scan_angle_range,
                'scan_downsample': scan_downsample,
                'scan_range': scan_range,
                'refresh_initial_path': refresh_initial_path,
                'flip_angle': flip_angle,
            }
        ],
        output='screen')   
    
    #===============================================定义启动文件========================================================

    ld = LaunchDescription()

    ld.add_action(neupan_server_node)

    return ld