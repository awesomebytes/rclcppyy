#include "boundary_kernel.hpp"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <sys/types.h>
#include <unistd.h>

namespace {

std::uint64_t parse_unsigned(const char * value, const char * name)
{
  if (value[0] == '-') {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  errno = 0;
  char * end = nullptr;
  const auto parsed = std::strtoull(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0') {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  return static_cast<std::uint64_t>(parsed);
}

std::uint64_t process_cpu_ns()
{
  timespec value{};
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_PROCESS_CPUTIME_ID) failed");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "usage: aot_boundary_worker ITERATIONS SEED RUN_TOKEN\n";
    return 2;
  }

  try {
    const auto iterations = parse_unsigned(argv[1], "iterations");
    const auto seed = parse_unsigned(argv[2], "seed");
    const std::string run_token(argv[3]);
    if (iterations == 0 || run_token.empty()) {
      throw std::invalid_argument("iterations and run token must be non-empty");
    }

    const auto warmup_iterations = std::min<std::uint64_t>(iterations, 10000ULL);
    volatile auto warmup = rclcppyy_boundary_benchmark::run_fused(
      warmup_iterations, seed);
    (void)warmup;

    const auto cpu_start = process_cpu_ns();
    const auto wall_start = std::chrono::steady_clock::now();
    const auto checksum = rclcppyy_boundary_benchmark::run_fused(iterations, seed);
    const auto wall_stop = std::chrono::steady_clock::now();
    const auto cpu_stop = process_cpu_ns();
    const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      wall_stop - wall_start).count();

    std::cout
      << "{\"schema\":\"rclcppyy.boundary-sample/v1\","
      << "\"run_token\":\"" << run_token << "\","
      << "\"variant\":\"aot-cpp\","
      << "\"pid\":" << static_cast<unsigned long long>(getpid()) << ","
      << "\"process_group_id\":" << static_cast<unsigned long long>(getpgrp()) << ","
      << "\"iterations\":" << static_cast<unsigned long long>(iterations) << ","
      << "\"checksum\":" << static_cast<unsigned long long>(checksum) << ","
      << "\"elapsed_ns\":" << elapsed_ns << ","
      << "\"cpu_time_ns\":" << static_cast<unsigned long long>(cpu_stop - cpu_start) << ","
      << "\"python_callback_count\":0,"
      << "\"backend\":{"
      << "\"schema\":\"rclcppyy.boundary-backend/v1\","
      << "\"backend\":\"aot-cpp\","
      << "\"execution_model\":\"standalone-optimized-executable\","
      << "\"evidence\":\"worker emitted result from independently compiled binary\"}}"
      << std::endl;
  } catch (const std::exception & error) {
    std::cerr << "aot boundary worker error: " << error.what() << "\n";
    return 2;
  }
  return 0;
}
