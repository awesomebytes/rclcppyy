cmake_minimum_required(VERSION 3.16)
project(rclcppyy_fusion_pipeline_aot LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(fusion_pipeline_aot fusion_pipeline_aot.cpp)
target_compile_features(fusion_pipeline_aot PRIVATE cxx_std_17)
target_compile_options(fusion_pipeline_aot PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(fusion_pipeline_aot PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
ament_target_dependencies(fusion_pipeline_aot rclcpp rmw std_msgs)
