#include "create3_state_machine/Create3StateMachine.hpp"

namespace create3_state_machine
{
Create3StateMachine::Create3StateMachine(const std::string& name) : Node(name), goal_reached(true), kp_(1.5)
{
  this->undock_client_ptr_ = rclcpp_action::create_client<UndockAction>(this, "undock");

  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&Create3StateMachine::goal_pose_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Create3StateMachine::odom_callback, this, std::placeholders::_1));

  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  points_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/points_test", 10);

  word_srv_ = this->create_service<create3_state_machine_msgs::srv::String>("input_word", std::bind(&Create3StateMachine::word_srv_callback, this, std::placeholders::_1,  std::placeholders::_2));

  // for(int i = 0; i < 6; ++i){
  //   geometry_msgs::msg::PoseStamped msg;
  //   msg.header.frame_id = "odom";
  //   msg.pose.position.x = poses[i*2] * 0.005;
  //   msg.pose.position.y = poses[i*2 + 1] * 0.005;
  //   msg.pose.orientation.w = 1;
  //   points_pub_->publish(msg);

  //   rclcpp::sleep_for(std::chrono::seconds(2));
  // }
}

Create3StateMachine::~Create3StateMachine() = default;

void Create3StateMachine::word_srv_callback(
    const std::shared_ptr<create3_state_machine_msgs::srv::String::Request> request,
    std::shared_ptr<create3_state_machine_msgs::srv::String::Response> response)
{
  if(request->word == "S"){
    response->result = "Executing...";
    send_goal_undock();
  }else{
    response->result = "Sadly requested letters are not implemented. Available: S";
  }
}

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
  std::vector<double> poses = { 630.0, 567.0, 567.0, 630.0, 567.0, 630.0, 472.0, 662.0, 472.0, 662.0, 346.0,
                                662.0, 346.0, 662.0, 252.0, 630.0, 252.0, 630.0, 189.0, 567.0, 189.0, 567.0,
                                189.0, 504.0, 189.0, 504.0, 220.0, 441.0, 220.0, 441.0, 252.0, 410.0, 252.0,
                                410.0, 315.0, 378.0, 315.0, 378.0, 504.0, 315.0, 504.0, 315.0, 567.0, 284.0,
                                567.0, 284.0, 598.0, 252.0, 598.0, 252.0, 630.0, 189.0, 630.0, 189.0, 630.0,
                                94.5,  630.0, 94.5,  567.0, 31.5,  567.0, 31.5,  472.0, 0.0,   472.0, 0.0,
                                346.0, 0.0,   346.0, 0.0,   252.0, 31.5,  252.0, 31.5,  189.0, 94.5 };

  std::map<char, std::vector<double>> my_map;

  my_map['S'] = poses;

  double dx = my_map['S'][iter * 2] * 0.005 - curr_pose.position.x;
  double dy = my_map['S'][iter * 2 + 1] * 0.005 - curr_pose.position.y;
  double distance = std::hypot(dx, dy);

  if (distance < 0.1)
  {
    ++iter;
    if (iter >= 38)
    {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached = true;
    }
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
  if (abs(error) < 0.05)
    cmd_vel.linear.x = 0.5;
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
