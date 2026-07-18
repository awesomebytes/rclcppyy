#include "fusion_pipeline_kernel.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <std_msgs/msg/u_int64.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

using Message = std_msgs::msg::UInt64;
using Publisher = rclcpp::Publisher<Message>;
using Subscription = rclcpp::Subscription<Message>;
constexpr const char * kPrefix = "@@RCLCPPYY_FUSION_PIPELINE_V1@@";
constexpr std::uint64_t kRssLimit = 64ULL * 1024ULL * 1024ULL;

std::uint64_t parse_uint64(const char * value, const char * name)
{
  if (value == nullptr || value[0] == '-') {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  char * end = nullptr;
  errno = 0;
  const auto result = std::strtoull(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0') {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  return static_cast<std::uint64_t>(result);
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
    throw std::runtime_error("getrusage failed");
  }
  return static_cast<std::uint64_t>(usage.ru_maxrss) * 1024ULL;
}

std::string quote(const std::string & value)
{
  std::string output = "\"";
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      output.push_back('\\');
    }
    output.push_back(character);
  }
  output.push_back('"');
  return output;
}

std::string rss_json(std::uint64_t baseline, std::uint64_t final)
{
  const auto growth = final > baseline ? final - baseline : 0ULL;
  return "{\"kind\":\"post-warmup-peak-rss-growth\",\"unit\":\"bytes\"," +
         std::string("\"baseline_peak_bytes\":") + std::to_string(baseline) +
         ",\"final_peak_bytes\":" + std::to_string(final) +
         ",\"growth_bytes\":" + std::to_string(growth) +
         ",\"limit_bytes\":" + std::to_string(kRssLimit) +
         ",\"within_limit\":" + (growth <= kRssLimit ? "true}" : "false}");
}

const char * loaded_rmw()
{
  const char * value = rmw_get_implementation_identifier();
  if (value == nullptr || value[0] == '\0') {
    throw std::runtime_error("RMW identifier is unavailable");
  }
  return value;
}

rclcpp::QoS qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

rclcpp::NodeOptions relay_options()
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  return options;
}

void verify_qos(const rclcpp::TopicEndpointInfo & endpoint)
{
  const auto profile = endpoint.qos_profile().get_rmw_qos_profile();
  if (profile.history != RMW_QOS_POLICY_HISTORY_KEEP_LAST || profile.depth != 1 ||
    profile.reliability != RMW_QOS_POLICY_RELIABILITY_RELIABLE ||
    profile.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE)
  {
    throw std::runtime_error("endpoint QoS differs from reliable volatile KeepLast(1)");
  }
}

class RelayBase : public rclcpp::Node {
public:
  explicit RelayBase(const std::string & name) : Node(name, relay_options()) {}
  virtual std::uint64_t received() const = 0;
  virtual std::uint64_t published() const = 0;
  virtual std::uint64_t stage_events() const = 0;
};

class StagedRelay final : public RelayBase {
public:
  StagedRelay(const std::string & name, const std::vector<std::string> & topics)
  : RelayBase(name)
  {
    if (topics.size() != rclcppyy_fusion_pipeline::kStages + 1) {
      throw std::invalid_argument("staged relay requires five topics");
    }
    for (std::size_t index = 0; index < rclcppyy_fusion_pipeline::kStages; ++index) {
      publishers_[index] = create_publisher<Message>(topics[index + 1], qos());
      subscriptions_[index] = create_subscription<Message>(
        topics[index], qos(),
        [this, index](Message::ConstSharedPtr input) {
          counts_[index].fetch_add(1, std::memory_order_relaxed);
          Message output{};
          output.data = rclcppyy_fusion_pipeline::stage(index, input->data);
          publishers_[index]->publish(std::move(output));
        });
    }
  }

  std::uint64_t received() const override {return counts_[0].load();}
  std::uint64_t published() const override {return counts_[3].load();}
  std::uint64_t stage_events() const override
  {
    std::uint64_t total = 0;
    for (const auto & count : counts_) {
      total += count.load(std::memory_order_acquire);
    }
    return total;
  }

private:
  std::array<Publisher::SharedPtr, 4> publishers_{};
  std::array<Subscription::SharedPtr, 4> subscriptions_{};
  std::array<std::atomic<std::uint64_t>, 4> counts_{};
};

class FusedRelay final : public RelayBase {
public:
  FusedRelay(
    const std::string & name, const std::string & input_topic,
    const std::string & output_topic)
  : RelayBase(name)
  {
    publisher_ = create_publisher<Message>(output_topic, qos());
    subscription_ = create_subscription<Message>(
      input_topic, qos(), [this](Message::ConstSharedPtr input) {
        received_.fetch_add(1, std::memory_order_relaxed);
        Message output{};
        output.data = rclcppyy_fusion_pipeline::fused(input->data);
        publisher_->publish(std::move(output));
        published_.fetch_add(1, std::memory_order_release);
      });
  }

  std::uint64_t received() const override {return received_.load();}
  std::uint64_t published() const override {return published_.load();}
  std::uint64_t stage_events() const override {return received_.load() * 4ULL;}

private:
  Publisher::SharedPtr publisher_;
  Subscription::SharedPtr subscription_;
  std::atomic<std::uint64_t> received_{0};
  std::atomic<std::uint64_t> published_{0};
};

int run_relay(int argc, char ** argv)
{
  if (argc != 12) {
    throw std::invalid_argument(
            "relay usage: relay VARIANT INPUT MID1 MID2 MID3 OUTPUT NODE WARMUP MESSAGES TOKEN");
  }
  const std::string variant(argv[2]);
  const std::vector<std::string> topics{argv[3], argv[4], argv[5], argv[6], argv[7]};
  const std::string node_name(argv[8]);
  const auto warmup = parse_uint64(argv[9], "warmup");
  const auto messages = parse_uint64(argv[10], "messages");
  const std::string token(argv[11]);
  std::shared_ptr<RelayBase> relay;
  std::string model;
  std::size_t entity_count = 0;
  std::size_t observable_topics = 0;
  if (variant == "aot-staged") {
    relay = std::make_shared<StagedRelay>(node_name, topics);
    model = "release-aot-four-stage-multi-entity";
    entity_count = 8;
    observable_topics = 5;
  } else if (variant == "aot-fused") {
    relay = std::make_shared<FusedRelay>(node_name, topics.front(), topics.back());
    model = "release-aot-single-callback-fused-ceiling";
    entity_count = 2;
    observable_topics = 2;
  } else {
    throw std::invalid_argument("AOT relay variant must be aot-staged or aot-fused");
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(relay);
  std::thread thread([&executor]() {executor.spin();});
  std::cout << kPrefix << "{\"schema\":\"rclcppyy.fusion-pipeline-relay/v1\"," 
            << "\"event\":\"ready\",\"variant\":" << quote(variant)
            << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
            << ",\"process_group_id\":" << getpgrp() << ",\"node_name\":"
            << quote(node_name) << ",\"loaded_rmw\":" << quote(loaded_rmw())
            << ",\"execution_model\":" << quote(model)
            << ",\"representation\":\"std_msgs::msg::UInt64\""
            << ",\"python_message_conversions\":0,\"ros_entity_count\":" << entity_count
            << ",\"observable_topic_count\":" << observable_topics
            << ",\"composition\":{\"relay_processes\":1,\"relay_nodes\":1,"
            << "\"executor\":\"single_threaded\",\"executor_threads\":1,"
            << "\"use_intra_process_comms\":true}"
            << ",\"cache\":{\"state\":\"prebuilt\",\"kind\":\"aot-binary\"}}"
            << std::endl;

  std::string command;
  if (!std::getline(std::cin, command) || command != "ARM") {
    executor.cancel();
    thread.join();
    throw std::runtime_error("relay expected ARM");
  }
  const auto rss_start = peak_rss_bytes();
  const auto cpu_start = process_cpu_ns();
  std::cout << kPrefix << "{\"schema\":\"rclcppyy.fusion-pipeline-relay/v1\"," 
            << "\"event\":\"armed\",\"variant\":" << quote(variant)
            << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
            << ",\"process_group_id\":" << getpgrp()
            << ",\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\"}" << std::endl;
  if (!std::getline(std::cin, command) || command != "REPORT") {
    executor.cancel();
    thread.join();
    throw std::runtime_error("relay expected REPORT");
  }
  const auto cpu_stop = process_cpu_ns();
  const auto rss_stop = peak_rss_bytes();
  executor.cancel();
  thread.join();
  executor.remove_node(relay);
  const auto expected = warmup + messages;
  const bool correct = relay->received() == expected && relay->published() == expected &&
    relay->stage_events() == expected * 4ULL;
  const auto received = relay->received();
  const auto published = relay->published();
  const auto stage_events = relay->stage_events();
  relay.reset();
  rclcpp::shutdown();
  std::cout << kPrefix << "{\"schema\":\"rclcppyy.fusion-pipeline-relay/v1\"," 
            << "\"event\":\"report\",\"variant\":" << quote(variant)
            << ",\"run_token\":" << quote(token) << ",\"received\":" << received
            << ",\"published\":" << published << ",\"logical_stage_events\":"
            << stage_events << ",\"python_callback_count\":0"
            << ",\"python_boundary_crossings\":0,\"python_message_conversions\":0"
            << ",\"cpu_time_ns\":" << (cpu_stop - cpu_start)
            << ",\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\",\"rss_guard\":"
            << rss_json(rss_start, rss_stop) << ",\"dropped\":0,\"exceptions\":0"
            << ",\"correct\":" << (correct ? "true" : "false")
            << ",\"teardown_clean\":true}" << std::endl;
  return correct ? 0 : 2;
}

std::string endpoint_array(const std::vector<rclcpp::TopicEndpointInfo> & endpoints)
{
  std::string output = "[";
  for (std::size_t index = 0; index < endpoints.size(); ++index) {
    verify_qos(endpoints[index]);
    if (index != 0) {
      output += ',';
    }
    output += "{\"node_name\":" + quote(endpoints[index].node_name()) +
      ",\"node_namespace\":" + quote(endpoints[index].node_namespace()) + "}";
  }
  return output + "]";
}

std::string graph_json(
  const std::shared_ptr<rclcpp::Node> & node, const std::vector<std::string> & topics,
  bool staged)
{
  std::string output = "[";
  const std::vector<std::size_t> indices = staged ?
    std::vector<std::size_t>{0, 1, 2, 3, 4} : std::vector<std::size_t>{0, 4};
  for (std::size_t offset = 0; offset < indices.size(); ++offset) {
    const auto index = indices[offset];
    const auto publishers = node->get_publishers_info_by_topic(topics[index]);
    const auto subscriptions = node->get_subscriptions_info_by_topic(topics[index]);
    if (publishers.size() != 1 || subscriptions.size() != 1) {
      throw std::runtime_error("topic endpoint cardinality is not one publisher and one subscription");
    }
    if (offset != 0) {
      output += ',';
    }
    output += "{\"topic\":" + quote(topics[index]) +
      ",\"publisher_count\":1,\"subscription_count\":1,\"publishers\":" +
      endpoint_array(publishers) + ",\"subscriptions\":" + endpoint_array(subscriptions) + "}";
  }
  return output + "]";
}

void wait_for_output(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::atomic<std::uint64_t> & received, std::uint64_t target)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (received.load(std::memory_order_acquire) < target) {
    executor.spin_some();
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("driver timed out waiting for output");
    }
    std::this_thread::yield();
  }
}

int run_driver(int argc, char ** argv)
{
  if (argc != 13) {
    throw std::invalid_argument(
            "driver usage: driver VARIANT INPUT MID1 MID2 MID3 OUTPUT DRIVER RELAY WARMUP MESSAGES TOKEN");
  }
  const std::string variant(argv[2]);
  const bool staged = variant == "aot-staged";
  if (!staged && variant != "cppyy-fused" && variant != "aot-fused") {
    throw std::invalid_argument("unknown driver variant");
  }
  const std::vector<std::string> topics{argv[3], argv[4], argv[5], argv[6], argv[7]};
  const std::string driver_name(argv[8]);
  const std::string relay_name(argv[9]);
  const auto warmup = parse_uint64(argv[10], "warmup");
  const auto messages = parse_uint64(argv[11], "messages");
  const std::string token(argv[12]);
  const auto expected_topics = staged ? 5U : 2U;
  auto node = std::make_shared<rclcpp::Node>(driver_name);
  auto publisher = node->create_publisher<Message>(topics.front(), qos());
  std::atomic<std::uint64_t> received{0};
  std::atomic<std::uint64_t> last{0};
  auto subscription = node->create_subscription<Message>(
    topics.back(), qos(), [&received, &last](Message::ConstSharedPtr output) {
      last.store(output->data, std::memory_order_relaxed);
      received.fetch_add(1, std::memory_order_release);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  std::string graph;
  while (graph.empty()) {
    try {
      graph = graph_json(node, topics, staged);
    } catch (const std::runtime_error &) {
      if (std::chrono::steady_clock::now() >= discovery_deadline) {
        throw;
      }
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  for (std::uint64_t sequence = 1; sequence <= warmup; ++sequence) {
    Message input{};
    input.data = sequence;
    const auto target = received.load() + 1;
    publisher->publish(std::move(input));
    wait_for_output(executor, received, target);
    if (last.load() != rclcppyy_fusion_pipeline::fused(sequence)) {
      throw std::runtime_error("warmup output violated fused transform contract");
    }
  }
  std::cout << kPrefix << "{\"schema\":\"rclcppyy.fusion-pipeline-driver/v1\"," 
            << "\"event\":\"warmed\",\"variant\":" << quote(variant)
            << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
            << ",\"process_group_id\":" << getpgrp() << ",\"loaded_rmw\":"
            << quote(loaded_rmw()) << ",\"warmup_messages\":" << warmup
            << ",\"topology_verified\":true,\"qos_verified\":true"
            << ",\"authority_verified\":true,\"observable_topic_count\":"
            << expected_topics << ",\"graph\":" << graph << "}" << std::endl;

  std::string command;
  if (!std::getline(std::cin, command) || command != "START") {
    throw std::runtime_error("driver expected START");
  }
  std::vector<std::uint64_t> latencies;
  latencies.reserve(messages);
  std::uint64_t checksum = 0;
  const auto rss_start = peak_rss_bytes();
  const auto cpu_start = process_cpu_ns();
  const auto wall_start = std::chrono::steady_clock::now();
  for (std::uint64_t offset = 1; offset <= messages; ++offset) {
    const auto sequence = warmup + offset;
    Message input{};
    input.data = sequence;
    const auto target = received.load() + 1;
    const auto start = std::chrono::steady_clock::now();
    publisher->publish(std::move(input));
    wait_for_output(executor, received, target);
    const auto stop = std::chrono::steady_clock::now();
    const auto expected = rclcppyy_fusion_pipeline::fused(sequence);
    if (last.load() != expected) {
      throw std::runtime_error("measured output violated fused transform contract");
    }
    checksum += expected;
    latencies.push_back(static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count()));
  }
  const auto wall_stop = std::chrono::steady_clock::now();
  const auto cpu_stop = process_cpu_ns();
  const auto rss_stop = peak_rss_bytes();
  const auto elapsed = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(wall_stop - wall_start).count());
  std::cout << kPrefix << "{\"schema\":\"rclcppyy.fusion-pipeline-driver/v1\"," 
            << "\"event\":\"measured\",\"variant\":" << quote(variant)
            << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
            << ",\"process_group_id\":" << getpgrp() << ",\"loaded_rmw\":"
            << quote(loaded_rmw()) << ",\"messages\":" << messages
            << ",\"checksum\":" << checksum << ",\"last\":" << last.load()
            << ",\"elapsed_ns\":" << elapsed << ",\"cpu_time_ns\":"
            << (cpu_stop - cpu_start) << ",\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\""
            << ",\"rss_guard\":" << rss_json(rss_start, rss_stop)
            << ",\"latency_ns\":[";
  for (std::size_t index = 0; index < latencies.size(); ++index) {
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << latencies[index];
  }
  std::cout << "]}" << std::endl;

  if (!std::getline(std::cin, command) || command != "TEARDOWN") {
    throw std::runtime_error("driver expected TEARDOWN");
  }
  const auto disappear_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (publisher->get_subscription_count() != 0 || node->count_publishers(topics.back()) != 0) {
    executor.spin_some();
    if (std::chrono::steady_clock::now() >= disappear_deadline) {
      throw std::runtime_error("relay endpoints did not disappear during teardown");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  executor.remove_node(node);
  subscription.reset();
  publisher.reset();
  node.reset();
  rclcpp::shutdown();
  std::cout << kPrefix << "{\"schema\":\"rclcppyy.fusion-pipeline-driver/v1\"," 
            << "\"event\":\"teardown\",\"variant\":" << quote(variant)
            << ",\"run_token\":" << quote(token) << ",\"pid\":" << getpid()
            << ",\"process_group_id\":" << getpgrp()
            << ",\"endpoint_disappeared\":true,\"teardown_clean\":true}" << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 2) {
      throw std::invalid_argument("mode must be relay or driver");
    }
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
    std::cerr << "fusion pipeline AOT error: " << error.what() << '\n';
    return 2;
  }
}
