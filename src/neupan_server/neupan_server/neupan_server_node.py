#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan, PointCloud2
from math import sin, cos, atan2
import numpy as np
from neupan.util import get_transform
import tf2_ros
import tf_transformations
import sensor_msgs.point_cloud2 as pc2
from neupan_interfaces.srv import NeupanCommand
from neupan import neupan

class NeupanSeverNode(Node):
    def __init__(self):
        super().__init__('neupan_server_node')

        # Declare and get parameters
        self.declare_parameter('config_file_path', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'laser_link')
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('marker_z', 1.0)
        self.declare_parameter('scan_angle_range', '-3.14 3.14')
        self.declare_parameter('scan_downsample', 1)
        self.declare_parameter('scan_range', '0.0 5.0')
        self.declare_parameter('dune_checkpoint', '')
        self.declare_parameter('refresh_initial_path', False)
        self.declare_parameter('flip_angle', False)

        self.planner_config_file = self.get_parameter('config_file_path').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.marker_z = self.get_parameter('marker_z').get_parameter_value().double_value
        scan_angle_range_para = self.get_parameter('scan_angle_range').get_parameter_value().string_value
        self.scan_angle_range = np.fromstring(scan_angle_range_para, dtype=np.float32, sep=' ')
        self.scan_downsample = self.get_parameter('scan_downsample').get_parameter_value().integer_value
        scan_range_para = self.get_parameter('scan_range').get_parameter_value().string_value
        self.scan_range = np.fromstring(scan_range_para, dtype=np.float32, sep=' ')
        self.dune_checkpoint = self.get_parameter('dune_checkpoint').get_parameter_value().string_value or None
        self.refresh_initial_path = self.get_parameter('refresh_initial_path').get_parameter_value().bool_value
        self.flip_angle = self.get_parameter('flip_angle').get_parameter_value().bool_value

        if not self.planner_config_file:
            self.get_logger().error("No planner config file provided! Please set the parameter 'config_file'")
            raise ValueError("Missing config file")
        # 初始化 neupan 实例
        pan = {'dune_checkpoint': self.dune_checkpoint}
        self.neupan_planner = neupan.init_from_yaml(self.planner_config_file, pan=pan)

        # Data initialization
        self.obstacle_points = None
        self.robot_state = None
        self.stop = False
        self.arrive = False
        self.goal = None

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/neupan_cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, '/neupan_plan', 10)
        self.ref_state_pub = self.create_publisher(Path, '/neupan_ref_state', 10)
        self.ref_path_pub = self.create_publisher(Path, '/neupan_initial_path', 10)
        self.point_markers_pub_dune = self.create_publisher(MarkerArray, '/dune_point_markers', 10)
        self.robot_marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.point_markers_pub_nrmp = self.create_publisher(MarkerArray, '/nrmp_point_markers', 10)
        
        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Path, '/initial_path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/neupan_goal', self.goal_callback, 10)

        # Main loop timer (50Hz)
        self.timer = self.create_timer(0.02, self.run)

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

    def run(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            trans = transform.transform.translation
            rot = transform.transform.rotation
            yaw = self.quat_to_yaw_list(rot)
            x, y = trans.x, trans.y
            self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info(
                f"Waiting for TF from {self.base_frame} to {self.map_frame}..."
            )
            return

        if self.robot_state is None:
            self.get_logger().warn("Waiting for robot state")
            return

        self.get_logger().info_once(f"Robot state received: {self.robot_state.tolist()}")

        if (
            len(self.neupan_planner.waypoints) >= 1
            and self.neupan_planner.initial_path is None
        ):
            self.neupan_planner.set_initial_path_from_state(self.robot_state)

        if self.neupan_planner.initial_path is None:
            self.get_logger().warn("Waiting for neupan initial path")
            return

        self.get_logger().info_once("Initial Path Received")
        self.ref_path_pub.publish(self.generate_path_msg(self.neupan_planner.initial_path))

        if self.obstacle_points is None:
            self.get_logger().warn("No obstacle points, only path tracking task will be performed")

        action, info = self.neupan_planner(self.robot_state, self.obstacle_points)

        self.stop = info["stop"]
        self.arrive = info["arrive"]

        if self.arrive:
            self.get_logger().info("Arrived at the target")

        self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
        self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
        self.vel_pub.publish(self.generate_twist_msg(action))

        dune_markers = self.generate_dune_points_markers_msg()
        if dune_markers:
            self.point_markers_pub_dune.publish(dune_markers)

        nrmp_markers = self.generate_nrmp_points_markers_msg()
        if nrmp_markers:
            self.point_markers_pub_nrmp.publish(nrmp_markers)

        robot_marker = self.generate_robot_marker_msg()
        if robot_marker:
            self.robot_marker_pub.publish(robot_marker)

        if self.stop:
            self.get_logger().info(
                f"Neupan stopped with min distance {self.neupan_planner.min_distance} "
                f"threshold {self.neupan_planner.collision_threshold}"
            )
    def scan_callback(self, scan_msg: LaserScan):
        if self.robot_state is None:
            return

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        points = []

        if self.flip_angle:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if (
                i % self.scan_downsample == 0
                and self.scan_range[0] <= distance <= self.scan_range[1]
                and self.scan_angle_range[0] < angle < self.scan_angle_range[1]
            ):
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        if not points:
            self.obstacle_points = None
            self.get_logger().info_once("No valid scan points")
            return

        point_array = np.hstack(points)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.lidar_frame,
                rclpy.time.Time()
            )

            trans = transform.transform.translation
            rot = transform.transform.rotation
            yaw = self.quat_to_yaw_list([rot.x, rot.y, rot.z, rot.w])
            x, y = trans.x, trans.y

            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points = rot_matrix @ point_array + trans_matrix
            self.get_logger().info_once("Scan obstacle points received")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info(
                f"Waiting for TF from {self.lidar_frame} to {self.map_frame}..."
            )
    def path_callback(self, path: Path):
        self.get_logger().info("Target path update")

        initial_point_list = []

        for p in path.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            theta = self.quat_to_yaw(p.pose.orientation)
            point = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(point)

        if self.neupan_planner.initial_path is None or self.refresh_initial_path:
            self.neupan_planner.set_initial_path(initial_point_list)

    def goal_callback(self, goal: PoseStamped):
        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        self.goal = np.array([[x], [y], [theta]])

        self.get_logger().info(f"Set neupan goal: {[x, y, theta]}")
        self.get_logger().info("Reference path update")

        self.neupan_planner.update_initial_path_from_goal(self.robot_state, self.goal)

    def generate_path_msg(self, path_list):
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for index, point in enumerate(path_list):
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = point[0, 0]
            pose.pose.position.y = point[1, 0]
            pose.pose.orientation.w = 1.0  # Orientation yaw not encoded here

            path_msg.poses.append(pose)

        return path_msg
    
    def generate_twist_msg(self, vel):
        if vel is None or self.stop or self.arrive:
            return Twist()

        speed = vel[0, 0]
        steer = vel[1, 0]

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steer

        return twist
    
    def generate_dune_points_markers_msg(self):
        marker_array = MarkerArray()

        if self.neupan_planner.dune_points is None:
            return None

        for index, point in enumerate(self.neupan_planner.dune_points.T):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size

            marker.color.a = 1.0
            marker.color.r = 160 / 255.0
            marker.color.g = 32 / 255.0
            marker.color.b = 240 / 255.0

            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.3

            marker.pose.orientation = Quaternion()

            marker_array.markers.append(marker)

        return marker_array
    
    def generate_nrmp_points_markers_msg(self):
        marker_array = MarkerArray()

        if self.neupan_planner.nrmp_points is None:
            return None

        for index, point in enumerate(self.neupan_planner.nrmp_points.T):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0

            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.3

            marker.pose.orientation = Quaternion()

            marker_array.markers.append(marker)

        return marker_array
    
    def generate_robot_marker_msg(self):
        if self.robot_state is None:
            return None

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        if self.neupan_planner.robot.shape == "rectangle":
            length = self.neupan_planner.robot.length
            width = self.neupan_planner.robot.width
            wheelbase = self.neupan_planner.robot.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z

            x = self.robot_state[0, 0]
            y = self.robot_state[1, 0]
            theta = self.robot_state[2, 0]

            if self.neupan_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0.0
            marker.pose.orientation = self.yaw_to_quat(theta)

        return marker

    @staticmethod
    def quat_to_yaw_list(quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw
    @staticmethod
    def yaw_to_quat(yaw):

        quater = Quaternion()

        quater.x = 0
        quater.y = 0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)

        return quater

    @staticmethod
    def quat_to_yaw(quater):

        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw
        
def main(args=None):
    rclpy.init(args=args)
    node = NeupanSeverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
