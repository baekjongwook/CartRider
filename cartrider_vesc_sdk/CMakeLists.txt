cmake_minimum_required(VERSION 3.8)
project(cartrider_vesc_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorCommandArray.msg"
  "msg/MotorState.msg"
  "msg/MotorStateArray.msg"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(hardware_node
  src/hardware_node.cpp
)

target_include_directories(hardware_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(hardware_node
  rclcpp
)

target_link_libraries(hardware_node
  ${cpp_typesupport_target}
)

add_executable(vesc_test_node
  src/vesc_test_node.cpp
)

target_include_directories(vesc_test_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(vesc_test_node
  rclcpp
  ament_index_cpp
)

target_link_libraries(vesc_test_node
  ${cpp_typesupport_target}
  yaml-cpp
)

install(TARGETS
  hardware_node
  vesc_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()