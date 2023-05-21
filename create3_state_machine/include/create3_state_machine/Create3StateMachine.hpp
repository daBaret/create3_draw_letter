#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/action/undock.hpp"

namespace create3_state_machine
{
class Create3StateMachine : public rclcpp::Node
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  Create3StateMachine(const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~Create3StateMachine();

private:
  using UndockAction = irobot_create_msgs::action::Undock;
  rclcpp_action::Client<UndockAction>::SharedPtr undock_client_ptr_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  void goal_pose_callback(const geometry_msgs::msg::PoseStamped & msg) const;

  void odom_callback(const nav_msgs::msg::Odometry & msg) const;

  void send_goal_undock();
};

}  // namespace create3_state_machine