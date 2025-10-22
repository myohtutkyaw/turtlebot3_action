cmake_minimum_required(VERSION 3.8)
project(midterm_ros2pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies (NO self find_package)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces from local action files
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Rotate.action"
  DEPENDENCIES builtin_interfaces
)

# Includes
include_directories(include)

# Nodes
add_executable(circle_publisher src/circle_publisher.cpp)
ament_target_dependencies(circle_publisher rclcpp geometry_msgs)

add_executable(odom_logger src/odom_logger.cpp)
ament_target_dependencies(odom_logger rclcpp nav_msgs tf2)

add_executable(square_service_server src/square_service_server.cpp)
ament_target_dependencies(square_service_server rclcpp std_srvs geometry_msgs nav_msgs tf2)

add_executable(square_service_client src/square_service_client.cpp)
ament_target_dependencies(square_service_client rclcpp std_srvs)

add_executable(abs_rotate_action_server src/abs_rotate_action_server.cpp)
ament_target_dependencies(abs_rotate_action_server rclcpp rclcpp_action geometry_msgs nav_msgs nav_msgs tf2 tf2_geometry_msgs)
rosidl_target_interfaces(abs_rotate_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(accu_rotate_action_server src/accu_rotate_action_server.cpp)
ament_target_dependencies(accu_rotate_action_server rclcpp rclcpp_action geometry_msgs nav_msgs)
rosidl_target_interfaces(accu_rotate_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(rotate_action_client src/rotate_action_client.cpp)
ament_target_dependencies(rotate_action_client rclcpp rclcpp_action)
rosidl_target_interfaces(rotate_action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Make sure generated headers exist before building nodes
add_dependencies(abs_rotate_action_server ${PROJECT_NAME}__rosidl_generator_cpp)
add_dependencies(accu_rotate_action_server ${PROJECT_NAME}__rosidl_generator_cpp)
add_dependencies(rotate_action_client ${PROJECT_NAME}__rosidl_generator_cpp)

install(TARGETS
  circle_publisher
  odom_logger
  square_service_client
  square_service_server
  abs_rotate_action_server
  rotate_action_client
  accu_rotate_action_server
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()