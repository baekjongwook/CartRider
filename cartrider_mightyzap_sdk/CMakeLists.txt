cmake_minimum_required(VERSION 3.8)
project(cartrider_mightyzap_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(hardware_node
  src/hardware_node.cpp
)

target_include_directories(hardware_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(hardware_node
  rclcpp
  std_msgs
)

add_executable(mightyzap_test_node
  src/mightyzap_test_node.cpp
)

target_include_directories(mightyzap_test_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(mightyzap_test_node
  rclcpp
  std_msgs
  ament_index_cpp
)

target_link_libraries(mightyzap_test_node
  yaml-cpp
)

install(TARGETS
  hardware_node
  mightyzap_test_node
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

ament_package()