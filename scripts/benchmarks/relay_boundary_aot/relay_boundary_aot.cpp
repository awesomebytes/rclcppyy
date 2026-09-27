#include "relay_boundary_kernel.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <std_msgs/msg/u_int64.hpp>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/types.h>
#include <sys/resource.h>
#include <unistd.h>

namespace {

using Message = std_msgs::msg::UInt64;
constexpr std::uint64_t kRssGuardLimitBytes = 64ULL * 1024ULL * 1024ULL;
constexpr const char * kProtocolPrefix = "@@RCLCPPYY_RELAY_BOUNDARY_V1@@";

class GraphIncomplete final : public std::runtime_error {
public:
  GraphIncomplete()
  : std::runtime_error("relay graph endpoint-info records are not complete") {}
};

std::uint64_t parse_uint64(const char * value, const char * name)
{
  if (value == nullptr || value[0] == '-') {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  char * end = nullptr;
  errno = 0;
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
    throw std::runtime_error("CLOCK_PROCESS_CPUTIME_ID failed");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

std::uint64_t peak_rss_bytes()
{
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0 || usage.ru_maxrss < 0) {
    throw std::runtime_error("getrusage(RUSAGE_SELF) failed");
  }
  return static_cast<std::uint64_t>(usage.ru_maxrss) * 1024ULL;
}

std::string rss_guard_json(std::uint64_t baseline, std::uint64_t final)
{
  const auto growth = final > baseline ? final - baseline : 0ULL;
  return "{\"kind\":\"post-warmup-peak-rss-growth\",\"unit\":\"bytes\"," +
         std::string("\"baseline_peak_bytes\":") + std::to_string(baseline) +
         ",\"final_peak_bytes\":" + std::to_string(final) +
         ",\"growth_bytes\":" + std::to_string(growth) +
         ",\"limit_bytes\":" + std::to_string(kRssGuardLimitBytes) +
         ",\"within_limit\":" + (growth <= kRssGuardLimitBytes ? "true}" : "false}");
}

std::string quote(const std::string & value)
{
  std::string result = "\"";
  for (const char character : value) {
    if (character == '\\' || character == '\"') {
      result.push_back('\\');
    }
    result.push_back(character);
  }
  result.push_back('\"');
  return result;
}

const char * loaded_rmw()
{
  const char * identifier = rmw_get_implementation_identifier();
  if (identifier == nullptr || identifier[0] == '\0') {
    throw std::runtime_error("loaded RMW identifier is unavailable");
  }
  return identifier;
}

rclcpp::QoS benchmark_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1));
}

void verify_qos(const rmw_qos_profile_t & profile)
{
  if (profile.history != RMW_QOS_POLICY_HISTORY_KEEP_LAST || profile.depth != 1 ||
    profile.reliability != RMW_QOS_POLICY_RELIABILITY_RELIABLE ||
    profile.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE)
  {
    throw std::runtime_error("actual endpoint QoS differs from reliable volatile KeepLast(1)");
  }
}

std::string endpoint_json(
  const rclcpp::TopicEndpointInfo & endpoint,
  const std::string & topic,
  const std::string & role,
  const std::string & expected_node_name,
  const std::string & ownership_evidence)
{
  const auto profile = endpoint.qos_profile().get_rmw_qos_profile();
  verify_qos(profile);
  return "{\"role\":" + quote(role) +
         ",\"topic\":" + quote(topic) +
         ",\"node_name\":" + quote(expected_node_name) +
         ",\"observed_node_name\":" + quote(endpoint.node_name()) +
         ",\"ownership_evidence\":" + quote(ownership_evidence) +
         ",\"node_namespace\":\"/\"" +
         ",\"observed_node_namespace\":" + quote(endpoint.node_namespace()) +
         ",\"history\":\"keep_last\",\"depth\":1," +
         "\"reliability\":\"reliable\",\"durability\":\"volatile\"}";
}

std::vector<std::string> graph_evidence(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & input_topic,
  const std::string & output_topic,
  const std::string & driver_name,
  const std::string & relay_name)
{
  const auto input_publishers = node->get_publishers_info_by_topic(input_topic);
  const auto input_subscriptions = node->get_subscriptions_info_by_topic(input_topic);
  const auto output_publishers = node->get_publishers_info_by_topic(output_topic);
  const auto output_subscriptions = node->get_subscriptions_info_by_topic(output_topic);
  if (input_publishers.size() > 1 || input_subscriptions.size() > 1 ||
    output_publishers.size() > 1 || output_subscriptions.size() > 1)
  {
    throw std::runtime_error("relay graph does not contain exactly four endpoints");
  }
  if (input_publishers.empty() || input_subscriptions.empty() ||
    output_publishers.empty() || output_subscriptions.empty())
  {
    throw GraphIncomplete();
  }
  if (input_publishers[0].node_name() != driver_name ||
    output_subscriptions[0].node_name() != driver_name)
  {
    throw std::runtime_error(
            "relay graph node ownership is incorrect: input pub=" +
            input_publishers[0].node_name() + ", input sub=" +
            input_subscriptions[0].node_name() + ", output pub=" +
            output_publishers[0].node_name() + ", output sub=" +
            output_subscriptions[0].node_name() + ", expected driver=" +
            driver_name + ", relay=" + relay_name);
  }
  const auto remote_owner_evidence = [&relay_name](const rclcpp::TopicEndpointInfo & endpoint) {
      if (endpoint.node_name() == relay_name) {
        return std::string("middleware-graph-owner");
      }
      if (endpoint.node_name() == "_NODE_NAME_UNKNOWN_" &&
        std::string(loaded_rmw()) == "rmw_cyclonedds_cpp")
      {
        return std::string("exact-process-pair-unique-topic");
      }
      throw std::runtime_error("remote relay endpoint owner is neither exact nor CycloneDDS-unknown");
    };
  return {
    endpoint_json(
      input_publishers[0], input_topic, "driver_publisher", driver_name,
      "middleware-graph-owner"),
    endpoint_json(
      input_subscriptions[0], input_topic, "relay_subscription", relay_name,
      remote_owner_evidence(input_subscriptions[0])),
    endpoint_json(
      output_publishers[0], output_topic, "relay_publisher", relay_name,
      remote_owner_evidence(output_publishers[0])),
    endpoint_json(
      output_subscriptions[0], output_topic, "driver_subscription", driver_name,
      "middleware-graph-owner"),
  };
}

class StagedRelay final : public rclcpp::Node {
public:
  StagedRelay(
    const std::string & node_name,
    const std::string & input_topic,
    const std::string & output_topic)
  : Node(node_name)
  {
    publisher_ = create_publisher<Message>(output_topic, benchmark_qos());
    subscription_ = create_subscription<Message>(
      input_topic, benchmark_qos(),
      [this](const Message::ConstSharedPtr input) {
        received_.fetch_add(1, std::memory_order_relaxed);
        checksum_.fetch_add(input->data, std::memory_order_relaxed);
        last_.store(input->data, std::memory_order_relaxed);
        Message output{};
        output.data = rclcppyy_relay_boundary::transform(input->data);
        processed_.fetch_add(1, std::memory_order_relaxed);
        publisher_->publish(std::move(output));
        published_.fetch_add(1, std::memory_order_release);
      });
  }

  std::uint64_t received() const {return received_.load(std::memory_order_acquire);}
  std::uint64_t processed() const {return processed_.load(std::memory_order_acquire);}
  std::uint64_t published() const {return published_.load(std::memory_order_acquire);}
  std::uint64_t checksum() const {return checksum_.load(std::memory_order_acquire);}
  std::uint64_t last() const {return last_.load(std::memory_order_acquire);}

private:
  rclcpp::Publisher<Message>::SharedPtr publisher_;
  rclcpp::Subscription<Message>::SharedPtr subscription_;
  std::atomic<std::uint64_t> received_{0};
  std::atomic<std::uint64_t> processed_{0};
  std::atomic<std::uint64_t> published_{0};
  std::atomic<std::uint64_t> checksum_{0};
  std::atomic<std::uint64_t> last_{0};
};

int run_relay(int argc, char ** argv)
{
  if (argc != 8) {
    throw std::invalid_argument(
            "relay usage: relay INPUT OUTPUT NODE WARMUP MESSAGES TOKEN");
  }
  const std::string input_topic(argv[2]);
  const std::string output_topic(argv[3]);
  const std::string node_name(argv[4]);
  const auto warmup = parse_uint64(argv[5], "warmup");
  const auto messages = parse_uint64(argv[6], "messages");
  const std::string token(argv[7]);
  if (messages == 0 || token.empty()) {
    throw std::invalid_argument("messages and token must be non-empty");
  }

  auto relay = std::make_shared<StagedRelay>(node_name, input_topic, output_topic);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(relay);
  std::thread executor_thread([&executor]() {executor.spin();});
  std::cout << kProtocolPrefix
            << "{\"schema\":\"rclcppyy.relay-boundary-relay-event/v1\","
            << "\"event\":\"ready\",\"variant\":\"aot-staged\","
            << "\"run_token\":" << quote(token) << ",\"pid\":" << getpid() << ","
            << "\"process_group_id\":" << getpgrp() << ",\"node_name\":"
            << quote(node_name) << ",\"loaded_rmw\":" << quote(loaded_rmw()) << ","
            << "\"execution_model\":\"conventional-release-aot-staged-relay\","
            << "\"cache\":{\"state\":\"prebuilt\",\"kind\":\"aot-binary\"}}"
            << std::endl;

  std::string command;
  if (!std::getline(std::cin, command) || command != "START") {
    executor.cancel();
    executor_thread.join();
    throw std::runtime_error("relay expected START control");
  }
  const auto rss_baseline = peak_rss_bytes();
  const auto cpu_start = process_cpu_ns();
  std::cout << kProtocolPrefix
            << "{\"schema\":\"rclcppyy.relay-boundary-relay-event/v1\","
            << "\"event\":\"armed\",\"variant\":\"aot-staged\","
            << "\"run_token\":" << quote(token) << ",\"pid\":" << getpid() << ","
            << "\"process_group_id\":" << getpgrp() << ","
            << "\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\"}" << std::endl;
  if (!std::getline(std::cin, command) || command != "REPORT") {
    executor.cancel();
    executor_thread.join();
    throw std::runtime_error("relay expected REPORT control");
  }
  const auto cpu_stop = process_cpu_ns();
  const auto rss_final = peak_rss_bytes();
  executor.cancel();
  executor_thread.join();
  executor.remove_node(relay);
  const auto received = relay->received();
  const auto processed = relay->processed();
  const auto published = relay->published();
  const auto checksum = relay->checksum();
  const auto last = relay->last();
  relay.reset();
  rclcpp::shutdown();
  const auto expected = warmup + messages;
  const bool correct = received == expected && processed == expected && published == expected;
  std::cout << kProtocolPrefix
            << "{\"schema\":\"rclcppyy.relay-boundary-relay-event/v1\","
            << "\"event\":\"report\",\"variant\":\"aot-staged\","
            << "\"run_token\":" << quote(token) << ",\"received\":" << received << ","
            << "\"processed\":" << processed << ",\"published\":" << published << ","
            << "\"checksum\":" << checksum << ",\"last\":" << last << ","
            << "\"python_callback_count\":0,\"python_boundary_crossings\":0,"
            << "\"cpu_time_ns\":" << (cpu_stop - cpu_start) << ","
            << "\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\","
            << "\"rss_guard\":" << rss_guard_json(rss_baseline, rss_final) << ","
            << "\"dropped\":0,\"exceptions\":0,\"correct\":"
            << (correct ? "true" : "false") << ",\"teardown_clean\":true}" << std::endl;
  return correct ? 0 : 2;
}

void wait_for_output(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::atomic<std::uint64_t> & received,
  std::uint64_t target)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (received.load(std::memory_order_acquire) < target) {
    executor.spin_some();
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("driver timed out waiting for a relay output");
    }
    std::this_thread::yield();
  }
}

int run_driver(int argc, char ** argv)
{
  if (argc != 9) {
    throw std::invalid_argument(
            "driver usage: driver INPUT OUTPUT DRIVER_NODE RELAY_NODE WARMUP MESSAGES TOKEN");
  }
  const std::string input_topic(argv[2]);
  const std::string output_topic(argv[3]);
  const std::string driver_name(argv[4]);
  const std::string relay_name(argv[5]);
  const auto warmup = parse_uint64(argv[6], "warmup");
  const auto messages = parse_uint64(argv[7], "messages");
  const std::string token(argv[8]);
  if (messages == 0 || token.empty()) {
    throw std::invalid_argument("messages and token must be non-empty");
  }

  auto node = std::make_shared<rclcpp::Node>(driver_name);
  auto publisher = node->create_publisher<Message>(input_topic, benchmark_qos());
  std::atomic<std::uint64_t> received{0};
  std::atomic<std::uint64_t> last{0};
  auto subscription = node->create_subscription<Message>(
    output_topic, benchmark_qos(),
    [&received, &last](const Message::ConstSharedPtr output) {
      last.store(output->data, std::memory_order_relaxed);
      received.fetch_add(1, std::memory_order_release);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (publisher->get_subscription_count() != 1 ||
    subscription->get_publisher_count() != 1)
  {
    executor.spin_some();
    if (std::chrono::steady_clock::now() >= discovery_deadline) {
      throw std::runtime_error("driver timed out waiting for the exact relay graph");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::vector<std::string> endpoints;
  while (endpoints.empty()) {
    try {
      endpoints = graph_evidence(
        node, input_topic, output_topic, driver_name, relay_name);
    } catch (const GraphIncomplete &) {
      if (std::chrono::steady_clock::now() >= discovery_deadline) {
        throw std::runtime_error(
                "driver timed out waiting for all four endpoint-info records");
      }
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  for (std::uint64_t sequence = 1; sequence <= warmup; ++sequence) {
    Message input{};
    input.data = sequence;
    const auto target = received.load(std::memory_order_acquire) + 1;
    publisher->publish(std::move(input));
    wait_for_output(executor, received, target);
    if (last.load(std::memory_order_acquire) != rclcppyy_relay_boundary::transform(sequence)) {
      throw std::runtime_error("warmup output violated the transform contract");
    }
  }
  std::cout << kProtocolPrefix
            << "{\"schema\":\"rclcppyy.relay-boundary-driver-event/v1\","
            << "\"event\":\"warmed\",\"run_token\":" << quote(token) << ","
            << "\"pid\":" << getpid() << ",\"process_group_id\":" << getpgrp() << ","
            << "\"warmup_messages\":" << warmup << ",\"loaded_rmw\":"
            << quote(loaded_rmw()) << "}" << std::endl;

  std::string command;
  if (!std::getline(std::cin, command) || command != "START") {
    throw std::runtime_error("driver expected START control");
  }

  std::vector<std::uint64_t> latencies;
  latencies.reserve(messages);
  std::uint64_t checksum = 0;
  const auto rss_baseline = peak_rss_bytes();
  const auto cpu_start = process_cpu_ns();
  const auto wall_start = std::chrono::steady_clock::now();
  for (std::uint64_t offset = 1; offset <= messages; ++offset) {
    const auto sequence = warmup + offset;
    Message input{};
    input.data = sequence;
    const auto target = received.load(std::memory_order_acquire) + 1;
    const auto message_start = std::chrono::steady_clock::now();
    publisher->publish(std::move(input));
    wait_for_output(executor, received, target);
    const auto message_stop = std::chrono::steady_clock::now();
    const auto expected = rclcppyy_relay_boundary::transform(sequence);
    if (last.load(std::memory_order_acquire) != expected) {
      throw std::runtime_error("measured output violated the transform contract");
    }
    checksum += expected;
    latencies.push_back(static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(message_stop - message_start).count()));
  }
  const auto wall_stop = std::chrono::steady_clock::now();
  const auto cpu_stop = process_cpu_ns();
  const auto rss_final = peak_rss_bytes();
  const auto elapsed_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(wall_stop - wall_start).count());

  const std::string rmw_identifier(loaded_rmw());
  std::cout << kProtocolPrefix
            << "{\"schema\":\"rclcppyy.relay-boundary-driver-event/v1\","
            << "\"event\":\"measured\",\"run_token\":" << quote(token) << ","
            << "\"pid\":" << getpid() << ",\"process_group_id\":" << getpgrp() << ","
            << "\"loaded_rmw\":" << quote(rmw_identifier) << ","
            << "\"execution_model\":\"identical-release-aot-closed-loop-driver\","
            << "\"messages\":" << messages << ",\"checksum\":" << checksum << ","
            << "\"last\":" << last.load() << ",\"elapsed_ns\":" << elapsed_ns << ","
            << "\"cpu_time_ns\":" << (cpu_stop - cpu_start) << ","
            << "\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\","
            << "\"rss_guard\":" << rss_guard_json(rss_baseline, rss_final) << ","
            << "\"topology_verified\":true,\"qos_verified\":true,"
            << "\"endpoints\":[";
  for (std::size_t index = 0; index < endpoints.size(); ++index) {
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << endpoints[index];
  }
  std::cout << "],\"latency_ns\":[";
  for (std::size_t index = 0; index < latencies.size(); ++index) {
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << latencies[index];
  }
  std::cout << "]}" << std::endl;

  if (!std::getline(std::cin, command) || command != "TEARDOWN") {
    throw std::runtime_error("driver expected TEARDOWN control");
  }
  executor.remove_node(node);
  subscription.reset();
  publisher.reset();
  node.reset();
  rclcpp::shutdown();
  std::cout << kProtocolPrefix
            << "{\"schema\":\"rclcppyy.relay-boundary-driver-event/v1\","
            << "\"event\":\"teardown\",\"run_token\":" << quote(token) << ","
            << "\"pid\":" << getpid() << ",\"process_group_id\":" << getpgrp() << ","
            << "\"teardown_clean\":true}" << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: relay_boundary_aot relay|driver ...\n";
    return 2;
  }
  try {
    rclcpp::init(0, nullptr);
    const std::string mode(argv[1]);
    if (mode == "relay") {
      return run_relay(argc, argv);
    }
    if (mode == "driver") {
      return run_driver(argc, argv);
    }
    throw std::invalid_argument("mode must be relay or driver");
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "relay boundary AOT error: " << error.what() << '\n';
    return 2;
  }
}
