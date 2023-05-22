#include "create3_state_machine/Create3StateMachine.hpp"
#include "create3_state_machine/letters.hpp"

namespace create3_state_machine
{
Create3StateMachine::Create3StateMachine(const std::string& name)
  : Node(name), goal_reached(true), kp_(1.5), current_state_(State::IDLE)
{
  this->undock_client_ptr_ = rclcpp_action::create_client<UndockAction>(this, "undock");
  this->dock_client_ptr_ = rclcpp_action::create_client<DockAction>(this, "dock");

  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&Create3StateMachine::goal_pose_callback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Create3StateMachine::odom_callback, this, _1));

  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  points_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/points_test", 10);

  word_srv_ = this->create_service<create3_state_machine_msgs::srv::String>(
      "input_word", std::bind(&Create3StateMachine::word_srv_callback, this, _1, _2));

  timer_state_machine_ = create_wall_timer(std::chrono::seconds(1), [this]() { state_machine(); });
}

Create3StateMachine::~Create3StateMachine() = default;

void Create3StateMachine::word_srv_callback(
    const std::shared_ptr<create3_state_machine_msgs::srv::String::Request> request,
    std::shared_ptr<create3_state_machine_msgs::srv::String::Response> response)
{
  bool letter_exists = true;
  for (char& letter : request->word)
  {
    letter_exists &= (letters.find(letter) == letters.end()) ? false : true;
  }
  if (letter_exists)
  {
    response->result = "Executing...";
    word_to_draw_ = request->word;
    current_state_ = State::UNDOCK;
  }
  else
  {
    response->result = "Sadly requested letters are not implemented. Available: SEVN";
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
    twist_msg = compute_twist(msg.pose.pose);
    twist_pub_->publish(twist_msg);
  }
}

geometry_msgs::msg::Twist Create3StateMachine::compute_twist(geometry_msgs::msg::Pose curr_pose)
{
  double dx, dy, distance;

  double scale = 0.001;
  double y_offset = (letter_size * word_iter_ - letter_size * word_to_draw_.size() / 2) * scale;
  double x_offset = 2;

  if (curr_letter == '!')
  {
    dx = letters[curr_letter][iter * 2] - curr_pose.position.x;
    dy = letters[curr_letter][iter * 2 + 1] - curr_pose.position.y;
    distance = std::hypot(dx, dy);
  }
  else
  {
    dx = letters[curr_letter][iter * 2] * scale- x_offset - curr_pose.position.x;
    dy = letters[curr_letter][iter * 2 + 1] * scale + y_offset - curr_pose.position.y;
    distance = std::hypot(dx, dy);
  }

  if (distance < 0.1)
  {
    ++iter;
    if (iter >= letters[curr_letter].size() / 2)
    {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached = true;
      iter = 0;
      current_state_ = (current_state_ == State::NEXT_LETTER_WAIT) ? State::NEXT_LETTER : State::DOCK;
      return cmd_vel;
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
  undock_options.result_callback = std::bind(&Create3StateMachine::result_undock_callback, this, _1);
  auto undock_msg = UndockAction::Goal();

  this->undock_client_ptr_->async_send_goal(undock_msg, undock_options);
}

void Create3StateMachine::send_goal_dock()
{
  if (!this->dock_client_ptr_->wait_for_action_server())
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto dock_options = rclcpp_action::Client<DockAction>::SendGoalOptions();
  dock_options.result_callback = std::bind(&Create3StateMachine::result_dock_callback, this, _1);
  dock_options.goal_response_callback = std::bind(&Create3StateMachine::goal_dock_response_callback, this, _1);
  auto dock_msg = DockAction::Goal();

  this->dock_client_ptr_->async_send_goal(dock_msg, dock_options);
}

void Create3StateMachine::result_undock_callback(
    const rclcpp_action::ClientGoalHandle<UndockAction>::WrappedResult& result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Undocked!");
      current_state_ = State::NEXT_LETTER;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Undocking Failed...");
      return;
  }
}

void Create3StateMachine::goal_dock_response_callback(
    std::shared_future<rclcpp_action::ClientGoalHandle<DockAction>::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server, trying again...");
    current_state_ = State::DOCK;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Docking goal was accepted!");
    current_state_ = State::DOCK_WAIT;
  }
}

void Create3StateMachine::result_dock_callback(const rclcpp_action::ClientGoalHandle<DockAction>::WrappedResult& result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Docked!");
      current_state_ = State::IDLE;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Docking Failed...");
      return;
  }
}

void Create3StateMachine::state_machine()
{
  switch (current_state_)
  {
    case State::UNDOCK:
      send_goal_undock();
      current_state_ = State::UNDOCK_WAIT;
      break;
    case State::UNDOCK_WAIT:
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for undock to finish");
      break;
    case State::DOCK:
      RCLCPP_INFO_STREAM(this->get_logger(), "Docking....");
      send_goal_dock();
      word_iter_ = 0;
      break;
    case State::DOCK_WAIT:
      RCLCPP_INFO_STREAM(this->get_logger(), "Docking....");
      break;
    case State::GO_TO_DOCK_WAIT:
      RCLCPP_INFO_STREAM(this->get_logger(), "Going to dock....");
      break;
    case State::GO_TO_DOCK:
      RCLCPP_INFO_STREAM(this->get_logger(), "Going to dock....");
      curr_letter = '!';
      goal_reached = false;
      current_state_ = State::GO_TO_DOCK_WAIT;
      break;
    case State::NEXT_LETTER_WAIT:
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Waiting for letter to finish");
      break;
    case State::NEXT_LETTER:
      if (word_iter_ == word_to_draw_.size())
        current_state_ = State::GO_TO_DOCK;
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Writing next letter....");
        goal_reached = false;
        curr_letter = word_to_draw_[word_iter_];
        ++word_iter_;
        current_state_ = State::NEXT_LETTER_WAIT;
      }
      break;

    default:
      break;
  }

}  // namespace create3_state_machine
}  // namespace create3_state_machine
