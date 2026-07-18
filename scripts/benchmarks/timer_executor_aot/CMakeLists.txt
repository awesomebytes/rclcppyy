cmake_minimum_required(VERSION 3.16)
project(rclcppyy_timer_executor_aot LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)

add_executable(timer_executor_aot timer_executor_aot.cpp)
target_compile_features(timer_executor_aot PRIVATE cxx_std_17)
target_compile_options(timer_executor_aot PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(timer_executor_aot rclcpp rmw)
