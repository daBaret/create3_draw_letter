#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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

  void send_goal_undock();
};

}  // namespace create3_state_machine