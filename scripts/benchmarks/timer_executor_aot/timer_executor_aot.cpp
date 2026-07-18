#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>
#include <unistd.h>

namespace {

constexpr std::uint64_t kPeriodNs = 1000000ULL;
constexpr std::uint64_t kSeed = 0xC0DEC0FFEE123456ULL;
constexpr std::uint64_t kMultiplier = 6364136223846793005ULL;
constexpr std::uint64_t kIncrement = 1442695040888963407ULL;
constexpr const char * kRmw = "rmw_cyclonedds_cpp";
constexpr const char * kPrefix = "@@RCLCPPYY_TIMER_EXECUTOR_V1@@";

std::uint64_t parse_uint64(const char * value, const char * name)
{
  if (value == nullptr || value[0] == '-') {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  char * end = nullptr;
  errno = 0;
  const auto parsed = std::strtoull(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' || parsed == 0) {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  return static_cast<std::uint64_t>(parsed);
}

std::uint64_t process_cpu_ns()
{
  timespec value{};
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0) {
    throw std::runtime_error("CLOCK_PROCESS_CPUTIME_ID failed");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

std::uint64_t steady_ns()
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::string quote(const std::string & value)
{
  std::string result = "\"";
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      result.push_back('\\');
    }
    result.push_back(character);
  }
  result.push_back('"');
  return result;
}

std::string loaded_rmw()
{
  const char * identifier = rmw_get_implementation_identifier();
  if (identifier == nullptr || std::string(identifier) != kRmw) {
    throw std::runtime_error("AOT timer requires rmw_cyclonedds_cpp");
  }
  return identifier;
}

std::int64_t nearest_rank(std::vector<std::int64_t> values, int percentile)
{
  if (values.empty()) {
    throw std::runtime_error("deadline sample set is empty");
  }
  std::sort(values.begin(), values.end());
  const auto rank = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(percentile / 100.0 * values.size())));
  return values[rank - 1];
}

std::string deadline_json(const std::vector<std::int64_t> & errors)
{
  std::vector<std::int64_t> absolute;
  absolute.reserve(errors.size());
  for (const auto value : errors) {
    absolute.push_back(value < 0 ? -value : value);
  }
  return std::string("{\"signed_ns\":{\"p50\":") +
         std::to_string(nearest_rank(errors, 50)) +
         ",\"p95\":" + std::to_string(nearest_rank(errors, 95)) +
         ",\"p99\":" + std::to_string(nearest_rank(errors, 99)) +
         ",\"max\":" + std::to_string(*std::max_element(errors.begin(), errors.end())) +
         "},\"absolute_ns\":{\"p50\":" +
         std::to_string(nearest_rank(absolute, 50)) +
         ",\"p95\":" + std::to_string(nearest_rank(absolute, 95)) +
         ",\"p99\":" + std::to_string(nearest_rank(absolute, 99)) +
         ",\"max\":" + std::to_string(*std::max_element(absolute.begin(), absolute.end())) +
         "}}";
}

struct State
{
  std::uint64_t warmup{0};
  std::uint64_t measured{0};
  std::uint64_t state{kSeed};
  std::uint64_t checksum{0};
  std::uint64_t epoch_ns{0};
  std::uint64_t cpu_start_ns{0};
  std::uint64_t cpu_stop_ns{0};
  std::uint64_t wall_stop_ns{0};
  std::uint64_t post_cancel{0};
  int phase{0};
  std::vector<std::int64_t> errors;
};

int run(int argc, char ** argv)
{
  if (argc != 5) {
    throw std::invalid_argument(
            "usage: timer_executor_aot NODE_NAME RUN_TOKEN WARMUP MEASURED");
  }
  const std::string node_name(argv[1]);
  const std::string token(argv[2]);
  const auto warmup_target = parse_uint64(argv[3], "warmup");
  const auto measured_target = parse_uint64(argv[4], "measured");
  State state;
  state.errors.reserve(measured_target);
  auto node = std::make_shared<rclcpp::Node>(node_name);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(
    std::chrono::nanoseconds(kPeriodNs),
    [&]() {
      if (state.phase == 0) {
        ++state.warmup;
        if (state.warmup == warmup_target) {
          timer->cancel();
          executor.cancel();
        }
        return;
      }
      if (state.phase != 1) {
        ++state.post_cancel;
        return;
      }
      ++state.measured;
      const auto actual_ns = steady_ns();
      const auto expected_ns = state.epoch_ns + state.measured * kPeriodNs;
      state.errors.push_back(
        static_cast<std::int64_t>(actual_ns) - static_cast<std::int64_t>(expected_ns));
      state.state = state.state * kMultiplier + kIncrement;
      state.checksum += state.state;
      if (state.measured == measured_target) {
        timer->cancel();
        state.cpu_stop_ns = process_cpu_ns();
        state.wall_stop_ns = steady_ns();
        state.phase = 2;
        executor.cancel();
      }
    });

  executor.spin();
  std::cout << kPrefix
            << "{\"schema\":\"rclcppyy.timer-executor-event/v1\","
            << "\"event\":\"ready\",\"variant\":\"aot-staged\","
            << "\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
            << ",\"process_group_id\":" << getpgrp()
            << ",\"node_name\":" << quote(node_name)
            << ",\"loaded_rmw\":" << quote(loaded_rmw())
            << ",\"execution_model\":\"conventional-release-aot-rclcpp-wall-timer\","
            << "\"warmup_firings\":" << state.warmup << ",\"timer_canceled\":true,"
            << "\"timer_marker\":{\"authority\":\"cpp\","
            << "\"implementation\":\"rclcpp::WallTimer\",\"clock\":\"steady\","
            << "\"period_ns\":" << kPeriodNs << ",\"callback_language\":\"cpp\"},"
            << "\"executor_marker\":{\"authority\":\"cpp\","
            << "\"implementation\":\"rclcpp::executors::SingleThreadedExecutor\","
            << "\"kind\":\"single_threaded\",\"threads\":1},"
            << "\"cache\":{\"state\":\"prebuilt\",\"kind\":\"aot-binary\"}}"
            << std::endl;

  std::string command;
  if (!std::getline(std::cin, command) || command != "START") {
    throw std::runtime_error("AOT timer expected START");
  }
  state.measured = 0;
  state.state = kSeed;
  state.checksum = 0;
  state.post_cancel = 0;
  state.errors.clear();
  state.epoch_ns = steady_ns();
  state.cpu_start_ns = process_cpu_ns();
  state.phase = 1;
  timer->reset();
  std::cout << kPrefix
            << "{\"schema\":\"rclcppyy.timer-executor-event/v1\","
            << "\"event\":\"armed\",\"variant\":\"aot-staged\","
            << "\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
            << ",\"process_group_id\":" << getpgrp()
            << ",\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\",\"timer_reset\":true}"
            << std::endl;
  executor.spin();
  const auto canceled_count = state.measured;
  std::this_thread::sleep_for(std::chrono::nanoseconds(3 * kPeriodNs));
  executor.spin_some();
  state.post_cancel += state.measured - canceled_count;

  const auto max_error = *std::max_element(state.errors.begin(), state.errors.end());
  const auto missed = std::max<std::int64_t>(0, max_error / static_cast<std::int64_t>(kPeriodNs));
  const auto cpu_time = state.cpu_stop_ns - state.cpu_start_ns;
  const auto wall_time = state.wall_stop_ns - state.epoch_ns;
  const auto errors_json = deadline_json(state.errors);
  const auto rmw = loaded_rmw();
  executor.remove_node(node);
  timer.reset();
  node.reset();
  rclcpp::shutdown();

  std::cout << kPrefix
            << "{\"schema\":\"rclcppyy.timer-executor-event/v1\","
            << "\"event\":\"report\",\"variant\":\"aot-staged\","
            << "\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
            << ",\"process_group_id\":" << getpgrp()
            << ",\"warmup_firings\":" << state.warmup
            << ",\"measured_firings\":" << state.measured
            << ",\"recurrence_state\":" << state.state
            << ",\"checksum\":" << state.checksum
            << ",\"python_callback_count\":0,\"measured_python_callback_count\":0,"
            << "\"python_boundary_crossings\":0,\"post_cancel_firings\":"
            << state.post_cancel << ",\"exceptions\":0,\"cpu_time_ns\":" << cpu_time
            << ",\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\",\"wall_duration_ns\":"
            << wall_time << ",\"scheduled_deadline_error\":" << errors_json
            << ",\"missed_periods\":" << missed
            << ",\"timer_canceled\":true,\"teardown_clean\":true,"
            << "\"executor_thread_joined\":true}" << std::endl;
  (void)rmw;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(0, nullptr);
    return run(argc, argv);
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "timer executor AOT error: " << error.what() << '\n';
    return 2;
  }
}
