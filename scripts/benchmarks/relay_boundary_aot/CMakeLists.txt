cmake_minimum_required(VERSION 3.16)
project(rclcppyy_relay_boundary_aot LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(relay_boundary_aot relay_boundary_aot.cpp)
target_compile_features(relay_boundary_aot PRIVATE cxx_std_17)
target_compile_options(relay_boundary_aot PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(relay_boundary_aot PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
ament_target_dependencies(relay_boundary_aot rclcpp rmw std_msgs)
