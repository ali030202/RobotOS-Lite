cmake_minimum_required(VERSION 3.8)
project(robotos_core)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotos_interfaces REQUIRED)

add_library(safety_state_machine src/safety_state_machine.cpp)
target_include_directories(safety_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(sim_node src/sim_node.cpp)
ament_target_dependencies(sim_node rclcpp sensor_msgs robotos_interfaces)
add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node safety_state_machine)
ament_target_dependencies(control_node rclcpp rclcpp_action sensor_msgs robotos_interfaces)

install(TARGETS safety_state_machine sim_node control_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_state_machine test/test_safety_state_machine.cpp)
  target_link_libraries(test_safety_state_machine safety_state_machine)
endif()
ament_package()
