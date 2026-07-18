cmake_minimum_required(VERSION 3.16)
project(rclcppyy_service_callback_aot LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(service_callback_aot service_callback_aot.cpp)
target_compile_features(service_callback_aot PRIVATE cxx_std_17)
target_compile_options(service_callback_aot PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(service_callback_aot rclcpp rmw std_srvs)
