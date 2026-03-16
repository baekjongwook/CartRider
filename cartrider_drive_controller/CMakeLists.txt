cmake_minimum_required(VERSION 3.8)
project(cartrider_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cartrider_rmd_sdk REQUIRED)

add_executable(rearbot_control_node
  src/rearbot_control_node.cpp
)

target_include_directories(rearbot_control_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(rearbot_control_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cartrider_rmd_sdk 
)

add_executable(rearbot_odom_node
  src/rearbot_odom_node.cpp
)

target_include_directories(rearbot_odom_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(rearbot_odom_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cartrider_rmd_sdk 
)

add_executable(teleop_keyboard
  src/teleop_keyboard.cpp
)

ament_target_dependencies(teleop_keyboard
  rclcpp
  geometry_msgs
)

add_executable(teleop_joystick
  src/teleop_joystick.cpp
)

ament_target_dependencies(teleop_joystick
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  rearbot_control_node
  rearbot_odom_node
  teleop_keyboard
  teleop_joystick
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

ament_package()