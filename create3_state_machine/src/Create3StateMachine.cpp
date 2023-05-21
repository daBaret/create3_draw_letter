#include "create3_state_machine/Create3StateMachine.hpp"

namespace create3_state_machine
{
Create3StateMachine::Create3StateMachine(const std::string& name) : Node(name)
{
  this->undock_client_ptr_ = rclcpp_action::create_client<UndockAction>(this, "undock");
  send_goal_undock();
}

Create3StateMachine::~Create3StateMachine() = default;

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
