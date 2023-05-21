#include "create3_state_machine/Create3StateMachine.hpp"

namespace create3_state_machine
{
Create3StateMachine::Create3StateMachine(const std::string& name)
  : Node(name), goal_reached(true), kp_(1.5)
{
  this->undock_client_ptr_ = rclcpp_action::create_client<UndockAction>(this, "undock");

  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&Create3StateMachine::goal_pose_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Create3StateMachine::odom_callback, this, std::placeholders::_1));

  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  points_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/points_test", 10);

  send_goal_undock();
}

Create3StateMachine::~Create3StateMachine() = default;

void Create3StateMachine::goal_pose_callback(const geometry_msgs::msg::PoseStamped& msg)
{
  goal_pose_ = msg.pose;
  goal_reached = false;
}

void Create3StateMachine::odom_callback(const nav_msgs::msg::Odometry& msg)
{
  if (!goal_reached)
  {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg = compute_twist(msg.pose.pose, this->now());
    twist_pub_->publish(twist_msg);
    
  }
}

geometry_msgs::msg::Twist Create3StateMachine::compute_twist(geometry_msgs::msg::Pose curr_pose, rclcpp::Time time)
{
  double dx = goal_pose_.position.x - curr_pose.position.x;
  double dy = goal_pose_.position.y - curr_pose.position.y;
  double distance = std::hypot(dx, dy);

  if (distance < 0.1)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    goal_reached = true;
  }

  double target_angle = std::atan2(dy, dx);
  tf2::Quaternion quaternion(curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z,
                             curr_pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  double error = target_angle - yaw;
  if (error > M_PI)
  {
    error -= 2 * M_PI;
  }
  else if (error < -M_PI)
  {
    error += 2 * M_PI;
  }

  // Simple P controller
  double angular_velocity = kp_ * error;

  geometry_msgs::msg::Twist cmd_vel;
  if(abs(error) < 0.05) cmd_vel.linear.x = 0.5;
  cmd_vel.angular.z = angular_velocity;

  return cmd_vel;
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
