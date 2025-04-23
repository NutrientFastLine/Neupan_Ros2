#ifndef NEUPAN_CONTROLLER_NEUPAN_CONTROLLER_HPP_
#define NEUPAN_CONTROLLER_NEUPAN_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/service_client.hpp"
#include "neupan_interfaces/srv/neupan_command.hpp"

namespace nav2_neupan_controller {

class NeupanController : public nav2_core::Controller {
public:
  NeupanController() = default;
  ~NeupanController() override = default;
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path &path) override;
  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

protected:
  // 存储插件名称
  std::string plugin_name_;
  // 存储坐标变换缓存指针，可用于查询坐标关系
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // 存储代价地图
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  // 存储节点指针
  nav2_util::LifecycleNode::SharedPtr node_;
  // 存储全局代价地图
//   nav2_costmap_2d::Costmap2D *costmap_;
  // 存储 setPlan 提供的全局路径
  nav_msgs::msg::Path global_plan_;

  //创建服务客户端
  // rclcpp::Node::SharedPtr raw_node_;
  // nav2_util::ServiceClient<neupan_interfaces::srv::NeupanCommand> client_;
  // std::unique_ptr<nav2_util::ServiceClient<neupan_interfaces::srv::NeupanCommand>> client_;
  // 独立的 Node 和 Service Client
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Client<neupan_interfaces::srv::NeupanCommand>::SharedPtr client_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;


  // 参数：最大线速度角速度
  double max_angular_speed_;
  double max_linear_speed_;

};

} // namespace nav2_neupan_controller

#endif // NEUPAN_CONTROLLER_NEUPAN_CONTROLLER_HPP_