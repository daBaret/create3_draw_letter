#include "rclcpp/rclcpp.hpp"
#include "create3_state_machine/Create3StateMachine.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<create3_state_machine::Create3StateMachine>("create3_state_machine");

  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}