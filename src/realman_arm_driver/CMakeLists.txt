cmake_minimum_required(VERSION 3.8)
project(realman_arm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmStatus.msg"
  "msg/JointStatus.msg"
  "srv/SetMode.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Include directories
include_directories(include)

# Create executable
add_executable(realman_arm_node
  src/main.cpp
  src/can_protocol.cpp
  src/realman_arm_node.cpp
)

ament_target_dependencies(realman_arm_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
)

# Link generated messages to executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(realman_arm_node "${cpp_typesupport_target}")

# Install executable
install(TARGETS
  realman_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts
install(PROGRAMS
  scripts/motor_setup.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
