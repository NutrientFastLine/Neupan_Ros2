#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import LaserScan
from neupan_interfaces.srv import NeupanCommand

from neupan import neupan

class NeupanSeverNode(Node):
    def __init__(self):
        super().__init__('neupan_server_node')

        # Declare and get planner config path
        # self.declare_parameter('config_file', '')
        # config_file = self.get_parameter('config_file').get_parameter_value().string_value
        # if not config_file:
        #     self.get_logger().error('No config file provided, please set ~config_file parameter.')
        #     raise RuntimeError("Missing config file")
        package_share_directory = get_package_share_directory('neupan_server')
        config_file = os.path.join(package_share_directory, 'config', 'neupan_controller.yaml')

        # 初始化 neupan 实例
        self.planner = neupan.init_from_yaml(config_file)

        # 创建 service
        self.srv = self.create_service(NeupanCommand, 'neupan_command', self.command_callback)
        self.get_logger().info('Neupan service [neupan_command] is ready.')

    def command_callback(self, request, response):
        try:
            # 读取机器人状态
            x, y, theta = request.pose.x, request.pose.y, request.pose.theta
            robot_state = np.array([[x], [y], [theta]])

            # 转换雷达数据为 obstacle 点
            scan = {
                'ranges': list(request.scan.ranges),
                'angle_min': request.scan.angle_min,
                'angle_max': request.scan.angle_max,
                'range_max': request.scan.range_max
            }

            points = self.planner.scan_to_point(robot_state, scan)

            action, info = self.planner(robot_state, points)

            twist = Twist()
            twist.linear.x = float(action[0, 0])
            twist.angular.z = float(action[1, 0])

            response.cmd_vel = twist
            response.success = True
            return response

        except Exception as e:
            self.get_logger().error(f"Error in Neupan planning: {e}")
            response.success = False
            return response


def main(args=None):
    rclpy.init(args=args)
    node = NeupanSeverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
