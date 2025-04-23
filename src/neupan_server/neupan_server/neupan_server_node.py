import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from neupan_interfaces.srv import NeupanCommand

from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from neupan import neupan
from neupan.util import get_transform

import numpy as np
from math import sin, cos, atan2

class NeupanServerNode(Node):
    def __init__(self):
        super().__init__('neupan_server_node')

        # parameter
        self.declare_parameter('config_file','')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser_link')
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('marker_z', 1.0)
        self.declare_parameter('scan_angle_range',[-3.14, 3.14])
        self.declare_parameter('scan_downsample', 1)
        self.declare_parameter('scan_range', [0.0, 0.5])
        self.declare_parameter('dune_checkpoint','')
        self.declare_parameter('refresh_initial_path', False)
        self.declare_parameter('flip_angle', False)

        self.planner_config_file_ = self.get_parameter('config_file').value
        self.map_frame_ = self.get_parameter('map_frame').value
        self.base_frame_ = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.marker_size_ = self.get_parameter('marker_size').value
        self.marker_z_ = self.get_parameter('marker_z').value
        self.scan_angle_range_= self.get_parameter('scan_angle_range').value
        self.scan_downsample_ = self.get_parameter('scan_downsample').value
        self.scan_range_ = self.get_parameter('scan_range').value
        self.dune_checkpoint_ = self.get_parameter('dune_checkpoint').value
        self.refresh_initial_path_ = self.get_parameter('refresh_initial_path').value
        self.flip_angle_ = self.get_parameter('flip_angle').value

        pan = {'dune_checkpoint': self.dune_checkpoint_}
        self.neupan_planner_ = neupan.init_from_yaml(
            self.planner_config_file_, pan=pan
        )

        # data
        self.obstacle_points_ = None #(2,n) n number of points
        self.robot_state_ = None #(3,1)[x, y, theta]
        self.stop_ = False

        #publisher
        self.vel_pub_ = self.create_publisher(Twist, 'neupan_cmd_vel', 10)
        self.plan_pub_ = self.create_publisher(Path, 'neupan_plan', 10)
        self.ref_state_pub_ = self.create_publisher(Path,'neupan_ref_state', 10)

        # for visulaization
        self.point_markers_pub_dune_ = self.create_publisher(MarkerArray, 'dune_point_markers', 10)
        self.robot_marker_pub_ = self.create_publisher(Marker, 'robot_marker', 10)
        self.point_marker_pub__nrmp_ = self.create_publisher(MarkerArray, 'nrmp_point_markers', 10)

        # tf listener
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # subscriber
        self.scan_sub_ = self.create_subscription(LaserScan,
                                                'scan',
                                                self.laserCallback,
                                                10)
        
        self.init_path_sub_ = self.create_subscription(Path,
                                                       'initial_path',
                                                       self.pathCallback,
                                                       10)
        
        self.neupan_goal_sub_ = self.create_subscription(PoseStamped,
                                                       'neupan_goal',
                                                       self.goalCallback,
                                                       10)
        
        self.srv = self.create_service(NeupanCommand, 'neupan_command', self.run)
        # self.timer_ = self.create_timer(0.02, self.run)
    
    def run(self, request, response):
        try:
            return self.handle_service_request(request, response)
        except Exception as e:
            self.get_logger().error(f"Error in Neupan run(): {e}")
            response.cmd_vel = Twist()
            response.success = False
            return response


    def handle_service_request(self, request, response):
        # 使用传入位置，不再重复做 TF 查询
        x = float(request.pose.x)
        y = float(request.pose.y)
        yaw = float(request.pose.theta)

        self.robot_state_ = np.array([x, y, yaw]).reshape(3, 1)

        # 安全检查：Planner 初始化 & 路径准备
        if self.robot_state_ is None:
            self.get_logger().warn("robot_state_ is None")
            response.cmd_vel = Twist()
            response.success = False
            return response

        self.get_logger().info(f"当前机器人坐标: {self.robot_state_.flatten().tolist()}")

        if len(self.neupan_planner_.waypoints) >= 1 and self.neupan_planner_.initial_path is None:
            self.neupan_planner_.set_initial_path_from_state(self.robot_state_)

        if self.neupan_planner_.initial_path is None:
            self.get_logger().warn("Waiting for Neupan initial path")
            response.cmd_vel = Twist()
            response.success = False
            return response
        
        self.get_logger().info(f"收到全局路径")

        if self.obstacle_points_ is None:
            self.get_logger().info(f"没有障碍点")

        # 主规划调用
        action, info = self.neupan_planner_(self.robot_state_, self.obstacle_points_)

        self.stop_ = info.get("stop", False)
        self.arrive_ = info.get("arrive", False)

        # 发布速度
        twist = self.generate_twist_msg(action)
        response.cmd_vel = twist
        response.success = True

        # 可选扩展信息（需扩展 srv）
        # response.arrive = self.arrive_
        # response.stop = self.stop_
        # response.min_distance = float(self.neupan_planner_.min_distance)
        self.get_logger().info(f"准备发布可视化信息")
        # 可视化信息发布
        self.publish_visualization(info, twist)

        return response


    def publish_visualization(self, info, twist):
        # self.ref_state_pub_.publish(self.generate_path_msg(info.get("ref_state_list", [])))
        # self.plan_pub_.publish(self.generate_path_msg(info.get("opt_state_list", [])))
        self.vel_pub_.publish(twist)
        # self.point_markers_pub_dune_.publish(self.generate_dune_points_markers_msg())
        # self.point_marker_pub__nrmp_.publish(self.generate_nrmp_points_markers_msg())
        # self.robot_marker_pub_.publish(self.generate_robot_marker_msg())
        self.get_logger().info(f"可视化信息发布完成")


    def laserCallback(self, msg):
        if self.robot_state_ is None:
            return None
        
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        points = []

        if self.flip_angle_:
            angles = np.flip(angles)
        
        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if(
                i % self.scan_downsample_ == 0
                and distance >= self.scan_range_[0]
                and distance <= self.scan_range_[1]
                and angle > self.scan_angle_range_[0]
                and angle < self.scan_angle_range_[1]
            ):
                point = np.array([[distance * cos(angle)],[distance * sin(angle)]])
                points.append(point)
        
        if len(points) == 0:
            self.obstacle_points_ = None
            self.get_logger().info("No valid scan points")
            return None

        point_array = np.hstack(points)

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer_.lookup_transform(self.map_frame_, self.laser_frame, now)
            t = trans.transform.translation
            
            x, y = t.x, t.y

            r = trans.transform.rotation
            rotation = [r.x, r.y, r.z, r.w]

            yaw = self.quat_to_yaw_list(rotation)
            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points_ = rot_matrix @ point_array + trans_matrix
            return self.obstacle_points_
            
        except TransformException as ex:
            self.get_logger().warn(f"Transform failed: {ex}")
            return

    def pathCallback(self, msg):
        self.get_logger().info("target path update")

        initial_point_list=[]

        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            theta = self.quat_to_yaw(p.pose.orientation)

            points = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(points)
        
        if self.neupan_planner_.initial_path is None or self.refresh_initial_path_:
            self.neupan_planner_.ipath.set_initial_path(initial_point_list)


    def goalCallback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = self.quat_to_yaw(msg.pose.orientation)

        self.goal_ = np.array([[x], [y], [theta]])

        self.get_logger().info(f"Set goal: {[x, y, theta]}")
        self.neupan_planner_.update_initial_path_from_goal(self.robot_state_, self.goal_)
    
    def generate_path_msg(self, path_list):
        path = Path()
        path.header.frame_id = self.map_frame_
        path.header.stamp = self.get_clock().now().to_msg()
    
        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame_
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = point[0, 0]
            ps.pose.position.y = point[1, 0]
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
    
        return path

    def generate_twist_msg(self, vel):
        msg = Twist()

        if vel is None or self.stop_ or self.arrive_:
            return msg

        msg.linear.x = float(vel[0, 0])
        msg.angular.z = float(vel[1, 0])
        self.get_logger().info(f"Action output types: {type(vel[0,0])}, {type(vel[1,0])}")
        self.get_logger().info(f"Action output types: {type(msg.linear.x)}, {type(msg.angular.z)}")
        return msg

    def generate_dune_points_markers_msg(self):
        marker_array = MarkerArray()

        if self.neupan_planner_.dune_points is None:
            return marker_array

        for index, point in enumerate(self.neupan_planner_.dune_points.T):
            marker = Marker()
            marker.header.frame_id = self.map_frame_
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = index
            marker.type = Marker.SPHERE
            marker.scale.x = self.marker_size_
            marker.scale.y = self.marker_size_
            marker.scale.z = self.marker_size_
            marker.color.a = 1.0
            marker.color.r = 160 / 255
            marker.color.g = 32 / 255
            marker.color.b = 240 / 255
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.3
            marker_array.markers.append(marker)

        return marker_array

    def generate_nrmp_points_markers_msg(self):
        marker_array = MarkerArray()

        if self.neupan_planner_.nrmp_points is None:
            return marker_array

        for index, point in enumerate(self.neupan_planner_.nrmp_points.T):
            marker = Marker()
            marker.header.frame_id = self.map_frame_
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = index
            marker.type = Marker.SPHERE
            marker.scale.x = self.marker_size_
            marker.scale.y = self.marker_size_
            marker.scale.z = self.marker_size_
            marker.color.a = 1.0
            marker.color.r = 255 / 255
            marker.color.g = 128 / 255
            marker.color.b = 0 / 255
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.3
            marker_array.markers.append(marker)

        return marker_array

    def generate_robot_marker_msg(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame_
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        if self.neupan_planner_.robot.shape == "rectangle":
            length = self.neupan_planner_.robot.length
            width = self.neupan_planner_.robot.width
            wheelbase = self.neupan_planner_.robot.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z_
            marker.type = Marker.CUBE

            x = self.robot_state_[0, 0]
            y = self.robot_state_[1, 0]
            theta = self.robot_state_[2, 0]

            if self.neupan_planner_.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0
            marker.pose.orientation = self.yaw_to_quat(theta)

        return marker

    def quat_to_yaw_list(self, quater):
        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw

    @staticmethod
    def quat_to_yaw(quater):
        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w
        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))
        return raw
    
    @staticmethod
    def yaw_to_quat(yaw):

        quater = Quaternion()

        quater.x = 0
        quater.y = 0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)

        return quater

def main(args=None):
    rclpy.init(args=args)
    node = NeupanServerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()
