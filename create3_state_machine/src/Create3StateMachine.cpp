#include "create3_state_machine/Create3StateMachine.hpp"

namespace create3_state_machine
{
Create3StateMachine::Create3StateMachine(const std::string& name) : Node(name)
{
  this->undock_client_ptr_ = rclcpp_action::create_client<UndockAction>(this, "undock");

  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&Create3StateMachine::goal_pose_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Create3StateMachine::odom_callback, this, std::placeholders::_1));

  send_goal_undock();
}

Create3StateMachine::~Create3StateMachine() = default;

void Create3StateMachine::goal_pose_callback(const geometry_msgs::msg::PoseStamped & msg) const{

  RCLCPP_INFO_STREAM(this->get_logger(), msg.pose.position.x);
}

void Create3StateMachine::odom_callback(const nav_msgs::msg::Odometry & msg) const{
  RCLCPP_INFO_STREAM(this->get_logger(), msg.pose.pose.position.x);
}

void Create3StateMachine::send_goal_undock()
{
  if (!this->undock_client_ptr_->wait_for_action_server())
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto undock_options = rclcpp_action::Client<UndockAction>::SendGoalOptions();
  auto undock_msg = UndockAction::Goal();

  this->undock_client_ptr_->async_send_goal(undock_msg, undock_options);

}

}  // namespace create3_state_machine
