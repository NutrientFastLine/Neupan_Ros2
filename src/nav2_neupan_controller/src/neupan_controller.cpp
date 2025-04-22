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

  // 声明并获取参数，设置最大线速度和最大角速度
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1));
  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
}

void NeupanController::cleanup() {
  RCLCPP_INFO(node_->get_logger(),
              "清理控制器：%s 类型为 nav2_custom_controller::CustomController",
              plugin_name_.c_str());
}

void NeupanController::activate() {
  RCLCPP_INFO(node_->get_logger(),
              "激活控制器：%s 类型为 nav2_custom_controller::CustomController",
              plugin_name_.c_str());
}

void NeupanController::deactivate() {
  RCLCPP_INFO(node_->get_logger(),
              "停用控制器：%s 类型为 nav2_custom_controller::CustomController",
              plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped NeupanController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {

  geometry_msgs::msg::TwistStamped cmd_vel;

  return cmd_vel;
}

void NeupanController::setSpeedLimit(const double &speed_limit,
                                     const bool &percentage) {
  (void)percentage;
  (void)speed_limit;
}

void NeupanController::setPlan(const nav_msgs::msg::Path &path) {
  global_plan_ = path;
}

} // namespace nav2_neupan_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_neupan_controller::NeupanController,nav2_core::Controller)