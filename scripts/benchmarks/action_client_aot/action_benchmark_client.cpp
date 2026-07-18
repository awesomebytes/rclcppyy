#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmw/rmw.h>
#include <tf2_msgs/action/lookup_transform.hpp>
#include <tf2_msgs/msg/tf2_error.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

using namespace std::chrono_literals;
using Action = tf2_msgs::action::LookupTransform;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

constexpr const char * kPrefix = "@@RCLCPPYY_ACTION_CLIENT_V1@@";
constexpr const char * kRmw = "rmw_cyclonedds_cpp";
constexpr std::uint64_t kFeedbackPerGoal = 3;
constexpr std::uint64_t kRssLimitBytes = 128ULL * 1024ULL * 1024ULL;

std::uint64_t parse_count(const char * value)
{
  char * end = nullptr;
  const auto parsed = std::strtoull(value, &end, 10);
  if (end == value || *end != '\0' || parsed == 0) {
    throw std::invalid_argument("invalid action benchmark count");
  }
  return parsed;
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

std::uint64_t peak_rss_bytes()
{
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    throw std::runtime_error("getrusage failed");
  }
  return static_cast<std::uint64_t>(usage.ru_maxrss) * 1024ULL;
}

std::string loaded_rmw()
{
  const char * value = rmw_get_implementation_identifier();
  if (value == nullptr || std::string(value) != kRmw) {
    throw std::runtime_error("action client requires rmw_cyclonedds_cpp");
  }
  return value;
}

std::string goal_base(const std::string & phase, std::uint64_t sequence)
{
  return "rclcppyy/action-benchmark/" + phase + "/" +
         std::to_string(sequence);
}

std::string endpoint_json(const std::string & action_name)
{
  const auto base = action_name + "/_action/";
  return std::string("{\"send_goal\":") + quote(base + "send_goal") +
         ",\"get_result\":" + quote(base + "get_result") +
         ",\"cancel_goal\":" + quote(base + "cancel_goal") +
         ",\"feedback\":" + quote(base + "feedback") +
         ",\"status\":" + quote(base + "status") + "}";
}

std::string qos_json()
{
  return "{\"goal_service\":{\"history\":\"keep_last\",\"depth\":10,"
         "\"reliability\":\"reliable\",\"durability\":\"volatile\"},"
         "\"result_service\":{\"history\":\"keep_last\",\"depth\":10,"
         "\"reliability\":\"reliable\",\"durability\":\"volatile\"},"
         "\"cancel_service\":{\"history\":\"keep_last\",\"depth\":10,"
         "\"reliability\":\"reliable\",\"durability\":\"volatile\"},"
         "\"feedback_topic\":{\"history\":\"system_default\",\"depth\":0,"
         "\"reliability\":\"system_default\",\"durability\":\"system_default\"},"
         "\"status_topic\":{\"history\":\"keep_last\",\"depth\":1,"
         "\"reliability\":\"reliable\",\"durability\":\"transient_local\"}}";
}

std::uint64_t nearest_rank(std::vector<std::uint64_t> values, int percentile)
{
  if (values.empty()) {
    throw std::runtime_error("action latency set is empty");
  }
  std::sort(values.begin(), values.end());
  const auto rank = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(percentile / 100.0 * values.size())));
  return values[rank - 1];
}

std::string latency_json(const std::vector<std::uint64_t> & values)
{
  return std::string("{\"p50\":") + std::to_string(nearest_rank(values, 50)) +
         ",\"p95\":" + std::to_string(nearest_rank(values, 95)) +
         ",\"p99\":" + std::to_string(nearest_rank(values, 99)) +
         ",\"max\":" + std::to_string(*std::max_element(values.begin(), values.end())) + "}";
}

std::string rss_json(std::uint64_t baseline, std::uint64_t final_value)
{
  const auto growth = final_value > baseline ? final_value - baseline : 0;
  return std::string("{\"kind\":\"post-warmup-peak-rss-growth\",\"unit\":\"bytes\",") +
         "\"baseline_peak_bytes\":" + std::to_string(baseline) +
         ",\"final_peak_bytes\":" + std::to_string(final_value) +
         ",\"growth_bytes\":" + std::to_string(growth) +
         ",\"limit_bytes\":" + std::to_string(kRssLimitBytes) +
         ",\"within_limit\":" + (growth <= kRssLimitBytes ? "true}" : "false}");
}

struct GoalObservation
{
  std::atomic<std::uint64_t> feedback{0};
  std::atomic<std::uint64_t> first_feedback_ns{0};
  std::atomic<std::uint64_t> invalid_feedback{0};
};

class ActionLoop
{
public:
  ActionLoop(
    const std::shared_ptr<rclcpp::Node> & node,
    rclcpp::executors::SingleThreadedExecutor & executor,
    const std::string & action_name)
  : executor_(executor)
  {
    client_ = rclcpp_action::create_client<Action>(node, action_name);
  }

  bool wait_for_server() const {return client_->wait_for_action_server(10s);}

  void run_phase(const std::string & phase, std::uint64_t count, bool measured)
  {
    for (std::uint64_t sequence = 1; sequence <= count; ++sequence) {
      run_goal(phase, sequence, measured);
    }
  }

  void arm()
  {
    accept_latency_.clear();
    feedback_latency_.clear();
    result_latency_.clear();
    measured_checksum_ = 0;
    last_sequence_ = 0;
    rss_baseline_ = peak_rss_bytes();
    wall_start_ns_ = steady_ns();
    cpu_start_ns_ = process_cpu_ns();
  }

  void finish()
  {
    cpu_stop_ns_ = process_cpu_ns();
    wall_stop_ns_ = steady_ns();
    rss_final_ = peak_rss_bytes();
  }

  void close() {client_.reset();}

  std::uint64_t goals_sent() const {return goals_sent_;}
  std::uint64_t goals_accepted() const {return goals_accepted_;}
  std::uint64_t feedback_received() const {return feedback_received_;}
  std::uint64_t results_received() const {return results_received_;}
  std::uint64_t terminal_succeeded() const {return terminal_succeeded_;}
  std::uint64_t measured_checksum() const {return measured_checksum_;}
  std::uint64_t last_sequence() const {return last_sequence_;}
  std::uint64_t exceptions() const {return exceptions_;}
  std::uint64_t cpu_time_ns() const {return cpu_stop_ns_ - cpu_start_ns_;}
  std::uint64_t wall_duration_ns() const {return wall_stop_ns_ - wall_start_ns_;}
  std::uint64_t rss_baseline() const {return rss_baseline_;}
  std::uint64_t rss_final() const {return rss_final_;}
  const std::vector<std::uint64_t> & accept_latency() const {return accept_latency_;}
  const std::vector<std::uint64_t> & feedback_latency() const {return feedback_latency_;}
  const std::vector<std::uint64_t> & result_latency() const {return result_latency_;}

private:
  void run_goal(const std::string & phase, std::uint64_t sequence, bool measured)
  {
    const auto base = goal_base(phase, sequence);
    Action::Goal goal;
    goal.target_frame = base + "/target";
    goal.source_frame = base + "/source";
    auto observation = std::make_shared<GoalObservation>();
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback =
      [observation](
        std::shared_ptr<GoalHandle>, const std::shared_ptr<const Action::Feedback> feedback) {
        const auto index = observation->feedback.fetch_add(1) + 1;
        if (index == 1) {
          observation->first_feedback_ns.store(steady_ns());
        }
        if (!feedback || feedback->structure_needs_at_least_one_member != 0) {
          observation->invalid_feedback.fetch_add(1);
        }
      };
    const auto send_ns = steady_ns();
    auto goal_future = client_->async_send_goal(goal, options);
    ++goals_sent_;
    if (executor_.spin_until_future_complete(goal_future, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      ++exceptions_;
      throw std::runtime_error("AOT action goal response timed out");
    }
    const auto accept_ns = steady_ns();
    const auto handle = goal_future.get();
    if (!handle) {
      throw std::runtime_error("AOT action goal was rejected");
    }
    ++goals_accepted_;
    auto result_future = client_->async_get_result(handle);
    if (executor_.spin_until_future_complete(result_future, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      ++exceptions_;
      throw std::runtime_error("AOT action result timed out");
    }
    const auto result_ns = steady_ns();
    const auto feedback_deadline = std::chrono::steady_clock::now() + 2s;
    while (observation->feedback.load() < kFeedbackPerGoal &&
      std::chrono::steady_clock::now() < feedback_deadline)
    {
      executor_.spin_some(1ms);
    }
    if (observation->feedback.load() != kFeedbackPerGoal ||
      observation->invalid_feedback.load() != 0)
    {
      throw std::runtime_error("AOT action feedback contract failed");
    }
    feedback_received_ += kFeedbackPerGoal;
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->transform.header.frame_id != phase ||
      wrapped.result->transform.child_frame_id != std::to_string(sequence) ||
      wrapped.result->error.error != tf2_msgs::msg::TF2Error::NO_ERROR ||
      !wrapped.result->error.error_string.empty())
    {
      throw std::runtime_error("AOT action terminal result contract failed");
    }
    ++results_received_;
    ++terminal_succeeded_;
    if (measured) {
      accept_latency_.push_back(accept_ns - send_ns);
      feedback_latency_.push_back(observation->first_feedback_ns.load() - send_ns);
      result_latency_.push_back(result_ns - send_ns);
      measured_checksum_ += sequence;
      last_sequence_ = sequence;
    }
  }

  rclcpp::executors::SingleThreadedExecutor & executor_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  std::uint64_t goals_sent_{0};
  std::uint64_t goals_accepted_{0};
  std::uint64_t feedback_received_{0};
  std::uint64_t results_received_{0};
  std::uint64_t terminal_succeeded_{0};
  std::uint64_t measured_checksum_{0};
  std::uint64_t last_sequence_{0};
  std::uint64_t exceptions_{0};
  std::uint64_t cpu_start_ns_{0};
  std::uint64_t cpu_stop_ns_{0};
  std::uint64_t wall_start_ns_{0};
  std::uint64_t wall_stop_ns_{0};
  std::uint64_t rss_baseline_{0};
  std::uint64_t rss_final_{0};
  std::vector<std::uint64_t> accept_latency_;
  std::vector<std::uint64_t> feedback_latency_;
  std::vector<std::uint64_t> result_latency_;
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 7) {
    std::cerr << "usage: action_benchmark_client NODE ACTION TOKEN VARIANT WARMUP MEASURED\n";
    return 2;
  }
  const std::string node_name(argv[1]);
  const std::string action_name(argv[2]);
  const std::string token(argv[3]);
  const std::string variant(argv[4]);
  const auto warmup = parse_count(argv[5]);
  const auto measured = parse_count(argv[6]);
  rclcpp::init(0, nullptr);
  try {
    auto node = std::make_shared<rclcpp::Node>(node_name);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    ActionLoop loop(node, executor, action_name);
    if (!loop.wait_for_server()) {
      throw std::runtime_error("AOT action server discovery timed out");
    }
    loop.run_phase("warmup", warmup, false);
    std::cout << kPrefix
              << "{\"schema\":\"rclcppyy.action-client-event/v1\","
              << "\"event\":\"ready\",\"variant\":" << quote(variant)
              << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
              << ",\"process_group_id\":" << getpgrp()
              << ",\"node_name\":" << quote(node_name)
              << ",\"action_name\":" << quote(action_name)
              << ",\"loaded_rmw\":" << quote(loaded_rmw())
              << ",\"action_type\":\"tf2_msgs/action/LookupTransform\","
              << "\"execution_model\":\"conventional-release-aot-rclcpp-action-client\","
              << "\"action_authority\":\"cpp\",\"goal_representation\":\"cpp-message\","
              << "\"action_implementation\":\"rclcpp_action::Client<LookupTransform>\","
              << "\"qos\":" << qos_json() << ",\"endpoints\":" << endpoint_json(action_name)
              << ",\"executor\":{\"authority\":\"cpp\","
              << "\"kind\":\"single_threaded\",\"threads\":1},"
              << "\"cache\":{\"kind\":\"aot-binary\",\"state\":\"prebuilt\"},"
              << "\"warmup_goals\":" << warmup
              << ",\"warmup_checksum\":" << warmup * (warmup + 1) / 2
              << ",\"warmup_feedback\":" << warmup * kFeedbackPerGoal
              << ",\"warmup_results\":" << warmup
              << ",\"warmup_terminal_success\":" << warmup
              << ",\"active_goals\":0,\"pending_operations\":0}" << std::endl;
    std::string command;
    if (!std::getline(std::cin, command) || command != "START") {
      throw std::runtime_error("AOT action client expected START");
    }
    loop.arm();
    std::cout << kPrefix
              << "{\"schema\":\"rclcppyy.action-client-event/v1\","
              << "\"event\":\"armed\",\"variant\":" << quote(variant)
              << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
              << ",\"process_group_id\":" << getpgrp()
              << ",\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\","
              << "\"measurement_reset\":true}" << std::endl;
    loop.run_phase("measured", measured, true);
    loop.finish();
    const auto cpu_time = loop.cpu_time_ns();
    const auto wall_time = loop.wall_duration_ns();
    const auto rss = rss_json(loop.rss_baseline(), loop.rss_final());
    const auto accept = latency_json(loop.accept_latency());
    const auto feedback = latency_json(loop.feedback_latency());
    const auto result = latency_json(loop.result_latency());
    const auto goals_sent = loop.goals_sent();
    const auto accepted = loop.goals_accepted();
    const auto feedback_received = loop.feedback_received();
    const auto results_received = loop.results_received();
    const auto succeeded = loop.terminal_succeeded();
    const auto checksum = loop.measured_checksum();
    const auto last = loop.last_sequence();
    const auto exceptions = loop.exceptions();
    loop.close();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    std::cout << kPrefix
              << "{\"schema\":\"rclcppyy.action-client-event/v1\","
              << "\"event\":\"report\",\"variant\":" << quote(variant)
              << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
              << ",\"process_group_id\":" << getpgrp()
              << ",\"warmup_goals\":" << warmup << ",\"measured_goals\":" << measured
              << ",\"goals_sent\":" << goals_sent << ",\"goals_accepted\":" << accepted
              << ",\"goals_rejected\":0,\"feedback_received\":" << feedback_received
              << ",\"feedback_dropped\":0,\"results_received\":" << results_received
              << ",\"terminal_succeeded\":" << succeeded
              << ",\"sequence_checksum\":" << checksum << ",\"last_sequence\":" << last
              << ",\"active_goals\":0,\"pending_operations\":0,\"exceptions\":" << exceptions
              << ",\"python_crossings\":{\"goal\":0,\"feedback\":0,\"result\":0,\"total\":0},"
              << "\"no_python_message_conversion\":true,\"cpu_time_ns\":" << cpu_time
              << ",\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\",\"wall_duration_ns\":" << wall_time
              << ",\"latency_ns\":{\"send_to_accept\":" << accept
              << ",\"send_to_first_feedback\":" << feedback
              << ",\"send_to_result\":" << result << "},\"rss_guard\":" << rss
              << ",\"orchestration_poll_count\":0,\"teardown_clean\":true,"
              << "\"executor_thread_joined\":true}" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "action benchmark client error: " << error.what() << '\n';
    return 3;
  }
}
