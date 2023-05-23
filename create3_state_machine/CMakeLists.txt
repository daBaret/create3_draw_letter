cmake_minimum_required(VERSION 3.8)
project(create3_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(create3_state_machine_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  irobot_create_msgs
  geometry_msgs
  nav_msgs
  create3_state_machine_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
  src/Create3StateMachine.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)


ament_package()
