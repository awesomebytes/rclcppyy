#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmw/rmw.h>
#include <tf2_msgs/action/lookup_transform.hpp>
#include <tf2_msgs/msg/tf2_error.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

using namespace std::chrono_literals;
using Action = tf2_msgs::action::LookupTransform;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

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
    throw std::runtime_error("action server requires rmw_cyclonedds_cpp");
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

struct Report
{
  std::uint64_t goals_received{0};
  std::uint64_t goals_accepted{0};
  std::uint64_t goals_rejected{0};
  std::uint64_t feedback_sent{0};
  std::uint64_t results_sent{0};
  std::uint64_t terminal_succeeded{0};
  std::uint64_t warmup_checksum{0};
  std::uint64_t measured_checksum{0};
  std::uint64_t active_goals{0};
  std::uint64_t exceptions{0};
  std::uint64_t cpu_start_ns{0};
  std::uint64_t cpu_stop_ns{0};
  std::uint64_t rss_baseline{0};
  std::uint64_t rss_final{0};
};

class BenchmarkServer
{
public:
  BenchmarkServer(
    std::shared_ptr<rclcpp::Node> node,
    std::string action_name,
    std::uint64_t warmup_goals,
    std::uint64_t measured_goals)
  : node_(std::move(node)),
    action_name_(std::move(action_name)),
    feedback_topic_(action_name_ + "/_action/feedback"),
    warmup_goals_(warmup_goals),
    measured_goals_(measured_goals)
  {
    server_ = rclcpp_action::create_server<Action>(
      node_, action_name_,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        return on_goal(std::move(goal));
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) { execute(handle); });
  }

  const Report & report() const {return report_;}

  bool all_goals_complete() const
  {
    return report_.results_sent == warmup_goals_ + measured_goals_ &&
           report_.active_goals == 0;
  }

  void close() {server_.reset();}

private:
  rclcpp_action::GoalResponse on_goal(std::shared_ptr<const Action::Goal> goal)
  {
    ++report_.goals_received;
    const bool warmup = report_.goals_received <= warmup_goals_;
    const auto sequence = warmup ? report_.goals_received :
      report_.goals_received - warmup_goals_;
    const std::string phase = warmup ? "warmup" : "measured";
    const auto base = goal_base(phase, sequence);
    const bool exact =
      goal && goal->target_frame == base + "/target" &&
      goal->source_frame == base + "/source" &&
      goal->source_time.sec == 0 && goal->source_time.nanosec == 0U &&
      goal->timeout.sec == 0 && goal->timeout.nanosec == 0U &&
      goal->target_time.sec == 0 && goal->target_time.nanosec == 0U &&
      goal->fixed_frame.empty() && !goal->advanced &&
      report_.goals_received <= warmup_goals_ + measured_goals_;
    if (!exact) {
      ++report_.goals_rejected;
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!warmup && sequence == 1) {
      report_.cpu_start_ns = process_cpu_ns();
    }
    ++report_.goals_accepted;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void execute(const std::shared_ptr<GoalHandle> & handle)
  {
    ++report_.active_goals;
    const auto goal = handle->get_goal();
    const bool warmup = report_.results_sent < warmup_goals_;
    const auto sequence = warmup ? report_.results_sent + 1 :
      report_.results_sent - warmup_goals_ + 1;
    const std::string phase = warmup ? "warmup" : "measured";
    const auto base = goal_base(phase, sequence);
    if (goal->target_frame != base + "/target" || goal->source_frame != base + "/source") {
      ++report_.exceptions;
      --report_.active_goals;
      return;
    }
    if (!feedback_matched_) {
      const auto deadline = std::chrono::steady_clock::now() + 5s;
      while (node_->count_subscribers(feedback_topic_) == 0 &&
        std::chrono::steady_clock::now() < deadline)
      {
        std::this_thread::sleep_for(1ms);
      }
      if (node_->count_subscribers(feedback_topic_) == 0) {
        ++report_.exceptions;
        --report_.active_goals;
        return;
      }
      std::this_thread::sleep_for(20ms);
      feedback_matched_ = true;
    }
    // Goal responses and feedback use independent action channels. Let the
    // client install this goal's handle before publishing its first feedback.
    std::this_thread::sleep_for(20ms);
    for (std::uint64_t remaining = kFeedbackPerGoal; remaining > 0; --remaining) {
      auto feedback = std::make_shared<Action::Feedback>();
      handle->publish_feedback(feedback);
      ++report_.feedback_sent;
      std::this_thread::sleep_for(10ms);
    }
    std::this_thread::sleep_for(20ms);
    auto result = std::make_shared<Action::Result>();
    result->transform.header.frame_id = phase;
    result->transform.child_frame_id = std::to_string(sequence);
    result->error.error = tf2_msgs::msg::TF2Error::NO_ERROR;
    result->error.error_string.clear();
    handle->succeed(result);
    if (warmup) {
      report_.warmup_checksum += sequence;
    } else {
      report_.measured_checksum += sequence;
    }
    ++report_.results_sent;
    ++report_.terminal_succeeded;
    --report_.active_goals;
    if (report_.results_sent == warmup_goals_) {
      report_.rss_baseline = peak_rss_bytes();
    }
    if (report_.results_sent == warmup_goals_ + measured_goals_) {
      report_.cpu_stop_ns = process_cpu_ns();
      report_.rss_final = peak_rss_bytes();
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::string action_name_;
  std::string feedback_topic_;
  std::uint64_t warmup_goals_;
  std::uint64_t measured_goals_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  bool feedback_matched_{false};
  Report report_;
};

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

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 7) {
    std::cerr << "usage: action_benchmark_server NODE ACTION TOKEN VARIANT WARMUP MEASURED\n";
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
    BenchmarkServer server(node, action_name, warmup, measured);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::cout << kPrefix
              << "{\"schema\":\"rclcppyy.action-server-event/v1\","
              << "\"event\":\"ready\",\"variant\":" << quote(variant)
              << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
              << ",\"process_group_id\":" << getpgrp()
              << ",\"node_name\":" << quote(node_name)
              << ",\"action_name\":" << quote(action_name)
              << ",\"loaded_rmw\":" << quote(loaded_rmw())
              << ",\"action_type\":\"tf2_msgs/action/LookupTransform\","
              << "\"execution_model\":\"conventional-release-aot-rclcpp-action-server\","
              << "\"action_authority\":\"cpp\","
              << "\"action_implementation\":\"rclcpp_action::Server<LookupTransform>\","
              << "\"goal_representation\":\"generated-cpp\","
              << "\"feedback_representation\":\"generated-cpp\","
              << "\"result_representation\":\"generated-cpp\","
              << "\"goal_id_representation\":\"generated-cpp\","
              << "\"envelope_representation\":\"generated-cpp\","
              << "\"cache\":{\"kind\":\"aot-binary\",\"state\":\"prebuilt\"},"
              << "\"qos\":" << qos_json()
              << ",\"endpoints\":" << endpoint_json(action_name)
              << ",\"executor\":{\"authority\":\"cpp\","
              << "\"kind\":\"single_threaded\",\"threads\":1}}" << std::endl;

    std::atomic<bool> stop{false};
    std::thread control([&stop]() {
      std::string command;
      if (std::getline(std::cin, command) && command == "STOP") {
        stop.store(true, std::memory_order_release);
      }
    });
    while (rclcpp::ok() && !stop.load(std::memory_order_acquire)) {
      executor.spin_some(2ms);
      std::this_thread::sleep_for(100us);
    }
    if (control.joinable()) {
      control.join();
    }
    if (!server.all_goals_complete()) {
      throw std::runtime_error("action server stopped before the exact goal matrix completed");
    }
    const auto report = server.report();
    server.close();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    std::cout << kPrefix
              << "{\"schema\":\"rclcppyy.action-server-event/v1\","
              << "\"event\":\"report\",\"variant\":" << quote(variant)
              << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
              << ",\"process_group_id\":" << getpgrp()
              << ",\"warmup_goals\":" << warmup << ",\"measured_goals\":" << measured
              << ",\"goals_received\":" << report.goals_received
              << ",\"goals_accepted\":" << report.goals_accepted
              << ",\"goals_rejected\":" << report.goals_rejected
              << ",\"feedback_sent\":" << report.feedback_sent
              << ",\"results_sent\":" << report.results_sent
              << ",\"terminal_succeeded\":" << report.terminal_succeeded
              << ",\"warmup_checksum\":" << report.warmup_checksum
              << ",\"measured_checksum\":" << report.measured_checksum
              << ",\"active_goals\":" << report.active_goals
              << ",\"pending_operations\":0,\"exceptions\":" << report.exceptions
              << ",\"cpu_time_ns\":" << report.cpu_stop_ns - report.cpu_start_ns
              << ",\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\","
              << "\"cpu_role\":\"drift_diagnostic_only\",\"rss_guard\":"
              << rss_json(report.rss_baseline, report.rss_final)
              << ",\"python_crossings\":{\"goal_decision\":0,"
              << "\"accepted_goal\":0,\"execute\":0,\"total\":0},"
              << "\"cpp_value_operations\":{\"known\":true,"
              << "\"goal_shared_handoffs\":0,\"goal_id_materializations\":0,"
              << "\"feedback_value_submissions\":0,\"result_value_submissions\":0,"
              << "\"adapter_message_deep_copies\":0},"
              << "\"boundary_evidence\":{\"exact_generated_cpp\":true,"
              << "\"python_message_conversions\":0,\"python_serialization_calls\":0,"
              << "\"adapter_cdr_roundtrips\":0,\"tripwires_armed\":false}"
              << ",\"teardown_clean\":true}" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "action benchmark server error: " << error.what() << '\n';
    return 3;
  }
}
