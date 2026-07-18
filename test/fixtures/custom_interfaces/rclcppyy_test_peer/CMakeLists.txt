cmake_minimum_required(VERSION 3.10)
project(rclcppyy_test_peer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcppyy_test_interfaces REQUIRED)

add_executable(interop_peer src/interop_peer.cpp)
target_compile_features(interop_peer PUBLIC cxx_std_17)
ament_target_dependencies(interop_peer
  rclcpp
  rclcpp_action
  rclcppyy_test_interfaces
)

install(TARGETS interop_peer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
