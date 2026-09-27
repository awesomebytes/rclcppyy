cmake_minimum_required(VERSION 3.16)
project(rclcppyy_service_client_aot LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_srvs REQUIRED)

set(RCLCPPYY_SERVICE_INTERFACE "std_srvs/srv/SetBool" CACHE STRING
  "Service interface exercised by the benchmark")

add_executable(service_client_aot service_client_aot.cpp)
if(RCLCPPYY_SERVICE_INTERFACE STREQUAL "std_srvs/srv/Trigger")
  target_compile_definitions(service_client_aot PRIVATE RCLCPPYY_BENCHMARK_TRIGGER=1)
elseif(NOT RCLCPPYY_SERVICE_INTERFACE STREQUAL "std_srvs/srv/SetBool")
  message(FATAL_ERROR "Unsupported RCLCPPYY_SERVICE_INTERFACE: ${RCLCPPYY_SERVICE_INTERFACE}")
endif()
target_compile_features(service_client_aot PRIVATE cxx_std_17)
target_compile_options(service_client_aot PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(service_client_aot rclcpp rmw std_srvs)
