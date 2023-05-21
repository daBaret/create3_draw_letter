#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "create3_state_machine_msgs/srv/string.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr points_pub_;
  rclcpp::Service<create3_state_machine_msgs::srv::String>::SharedPtr word_srv_;

  double kp_;
  int iter = 0;

  rclcpp::Time prev_time;

  geometry_msgs::msg::Pose goal_pose_;
  bool goal_reached;

  void goal_pose_callback(const geometry_msgs::msg::PoseStamped& msg);

  void odom_callback(const nav_msgs::msg::Odometry& msg);

  void send_goal_undock();

  void word_srv_callback(const std::shared_ptr<create3_state_machine_msgs::srv::String::Request> request,
                         std::shared_ptr<create3_state_machine_msgs::srv::String::Response> response);

  geometry_msgs::msg::Twist compute_twist(geometry_msgs::msg::Pose, rclcpp::Time time);
};

}  // namespace create3_state_machine