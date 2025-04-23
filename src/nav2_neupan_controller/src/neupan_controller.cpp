#include "nav2_neupan_controller/neupan_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace nav2_neupan_controller {
void NeupanController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;

  // auto base_node = rclcpp::Node::SharedPtr(node_->get_node_base_interface(), rclcpp::Node::get_node_deleter());
  // nav2_util::ServiceClient<neupan_interfaces::srv::NeupanCommand> client_("neupan_command", base_node);
  // nav2_util::LifecycleServiceClient<neupan_interfaces::srv::NeupanCommand> client("your_service_name", shared_from_this());
  auto raw_node = rclcpp::Node::make_shared("neupan_client_node");
  client_ = raw_node->create_client<neupan_interfaces::srv::NeupanCommand>("neupan_command");

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("initial_path", 10);
  // 声明并获取参数，设置最大线速度和最大角速度
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1));
  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);

  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    throw nav2_core::PlannerException("NeupanCommand service not available.");
  }

}

void NeupanController::cleanup() {
  RCLCPP_INFO(node_->get_logger(),
              "清理控制器：%s 类型为 nav2_custom_controller::CustomController",
              plugin_name_.c_str());
  path_pub_.reset();
}

void NeupanController::activate() {
  RCLCPP_INFO(node_->get_logger(),
              "激活控制器：%s 类型为 nav2_custom_controller::CustomController",
              plugin_name_.c_str());
  path_pub_->on_activate();
}

void NeupanController::deactivate() {
  RCLCPP_INFO(node_->get_logger(),
              "停用控制器：%s 类型为 nav2_custom_controller::CustomController",
              plugin_name_.c_str());
  path_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped NeupanController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {

  if (!client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "NeupanCommand service not ready");
    throw nav2_core::PlannerException("Service not ready");
  }

  auto request = std::make_shared<neupan_interfaces::srv::NeupanCommand::Request>();
  request->pose.x = pose.pose.position.x;
  request->pose.y = pose.pose.position.y;

  double yaw = tf2::getYaw(pose.pose.orientation);
  request->pose.theta = yaw;

  auto future = client_->async_send_request(request);

  //spin_until_future_complete 这里会阻塞nav2的线程，决定了service在插件里用不了

  if (rclcpp::spin_until_future_complete(raw_node_, future, std::chrono::milliseconds(100)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed");
    throw nav2_core::PlannerException("Failed to call Neupan service");
  }
  // // 使用单独 executor，避免 controller_server executor 冲突
  // rclcpp::executors::SingleThreadedExecutor exec;
  // exec.add_node(raw_node_);
  // if (exec.spin_until_future_complete(future, std::chrono::milliseconds(100)) !=
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_ERROR(node_->get_logger(), "Service call failed");
  //   throw nav2_core::PlannerException("Failed to call Neupan service");
  // }

  // auto response = future.get();

  // if (!response->success) {
  //   RCLCPP_WARN(node_->get_logger(), "Neupan planner failed to compute velocity");
  //   throw nav2_core::PlannerException("Neupan planner returned failure");
  // }

  geometry_msgs::msg::TwistStamped cmd_vel_stamped;
  cmd_vel_stamped.header.stamp = node_->now();
  cmd_vel_stamped.header.frame_id = "base_link";  // 或实际控制坐标系
  // cmd_vel_stamped.twist = response->cmd_vel;
  cmd_vel_stamped.twist.linear.x = 0.6;
  cmd_vel_stamped.twist.angular.z = 1.0;

  return cmd_vel_stamped;

}

void NeupanController::setSpeedLimit(const double &speed_limit,
                                     const bool &percentage) {
  (void)percentage;
  (void)speed_limit;
}

void NeupanController::setPlan(const nav_msgs::msg::Path &path) {
  global_plan_ = path;
  if (!global_plan_.poses.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Received global path with %zu poses, frame_id: %s",
                global_plan_.poses.size(), global_plan_.header.frame_id.c_str());
  }
  
  // global_plan_.header.frame_id = "map"; // 确保与 Python 中 frame 对应
  // global_plan_.header.stamp = node_->now();
  path_pub_->publish(global_plan_);
}

} // namespace nav2_neupan_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_neupan_controller::NeupanController,nav2_core::Controller)