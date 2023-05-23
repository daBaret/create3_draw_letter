#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "create3_state_machine_msgs/srv/string.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/action/dock_servo.hpp"

namespace create3_state_machine
{
enum class State
{
  IDLE,
  UNDOCK,
  UNDOCK_WAIT,
  DOCK,
  DOCK_WAIT,
  GO_TO_DOCK,
  GO_TO_DOCK_WAIT,
  NEXT_LETTER,
  NEXT_LETTER_WAIT
};

using namespace std::placeholders;

class Create3StateMachine : public rclcpp::Node
{
public:
  /*!
   * Constructor.
   */
  Create3StateMachine(const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~Create3StateMachine();

private:
  // Actions client defines
  using UndockAction = irobot_create_msgs::action::Undock;
  using DockAction = irobot_create_msgs::action::DockServo;
  rclcpp_action::Client<UndockAction>::SharedPtr undock_client_ptr_;
  rclcpp_action::Client<DockAction>::SharedPtr dock_client_ptr_;

  // Subscription and Publisher
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr points_pub_;
  rclcpp::Service<create3_state_machine_msgs::srv::String>::SharedPtr word_srv_;

  // State machine vars
  rclcpp::TimerBase::SharedPtr timer_state_machine_;
  State current_state_;

  // Path following vars
  geometry_msgs::msg::Pose goal_pose_;
  bool goal_reached = true;
  unsigned int word_iter_ = 0;
  std::string word_to_draw_;
  double kp_; // P value for rotation
  unsigned int path_iter_ = 0;
  char curr_letter_;

  // Poses used to dwar letter in rviz
  geometry_msgs::msg::PoseArray poses_msg_;

  // Callback for odom topic
  void odom_callback(const nav_msgs::msg::Odometry& msg);

  // Call undock action
  void send_goal_undock();
  // Call dock action
  void send_goal_dock();

  // Service callback for requested word
  void word_srv_callback(const std::shared_ptr<create3_state_machine_msgs::srv::String::Request> request,
                         std::shared_ptr<create3_state_machine_msgs::srv::String::Response> response);

  // This callback is if the undocking process was successfull or not
  void result_undock_callback(const rclcpp_action::ClientGoalHandle<UndockAction>::WrappedResult& result);
  // This callback is if the docking process was successfull or not
  void result_dock_callback(const rclcpp_action::ClientGoalHandle<DockAction>::WrappedResult& result);
  // This callback is if the docking request was accepted
  void goal_dock_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<DockAction>::SharedPtr> future);

  // State machine function, the timer calls it
  void state_machine();

  // Compute the twist command to track the path
  geometry_msgs::msg::Twist compute_twist(geometry_msgs::msg::Pose);
};

}  // namespace create3_state_machine