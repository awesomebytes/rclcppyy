#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

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
#include <vector>

#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

#ifdef RCLCPPYY_BENCHMARK_TRIGGER
using Service = std_srvs::srv::Trigger;
constexpr const char * kServiceType = "std_srvs/srv/Trigger";
constexpr const char * kCppServiceType = "std_srvs::srv::Trigger";
constexpr const char * kServerModel = "common-release-aot-trigger-server";
#else
using Service = std_srvs::srv::SetBool;
constexpr const char * kServiceType = "std_srvs/srv/SetBool";
constexpr const char * kCppServiceType = "std_srvs::srv::SetBool";
constexpr const char * kServerModel = "common-release-aot-setbool-server";
#endif
constexpr const char * kPrefix = "@@RCLCPPYY_SERVICE_CLIENT_V1@@";
constexpr std::uint64_t kRssLimitBytes = 64ULL * 1024ULL * 1024ULL;

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
         ",\"limit_bytes\":" + std::to_string(kRssLimitBytes) +
         ",\"within_limit\":" + (growth <= kRssLimitBytes ? "true}" : "false}");
}

const char * loaded_rmw()
{
  const char * value = rmw_get_implementation_identifier();
  if (value == nullptr || value[0] == '\0') {
    throw std::runtime_error("loaded RMW identifier is unavailable");
  }
  return value;
}

rclcpp::QoS service_qos()
{
  const auto profile = rmw_qos_profile_services_default;
  if (profile.history != RMW_QOS_POLICY_HISTORY_KEEP_LAST || profile.depth != 10 ||
    profile.reliability != RMW_QOS_POLICY_RELIABILITY_RELIABLE ||
    profile.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE)
  {
    throw std::runtime_error("service QoS is not reliable volatile KeepLast(10)");
  }
  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(profile), profile);
}

rclcpp::NodeOptions node_options()
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.enable_rosout(false);
  return options;
}

std::uint64_t response_code(bool request, const Service::Response & response)
{
#ifdef RCLCPPYY_BENCHMARK_TRIGGER
  (void)request;
  return (response.success ? 100ULL : 0ULL) + response.message.size();
#else
  return (response.success ? 100ULL : 0ULL) +
         (request ? 10ULL : 0ULL) + response.message.size();
#endif
}

void validate_response(bool request, const Service::Response & response)
{
#ifdef RCLCPPYY_BENCHMARK_TRIGGER
  (void)request;
  if (!response.success || response.message != "triggered") {
    throw std::runtime_error("Trigger response violated the benchmark contract");
  }
#else
  const std::string expected = request ? "enabled" : "disabled";
  if (response.success != request || response.message != expected) {
    throw std::runtime_error("SetBool response violated the benchmark contract");
  }
#endif
}

bool sequence_value(std::uint64_t sequence)
{
#ifdef RCLCPPYY_BENCHMARK_TRIGGER
  (void)sequence;
  return true;
#else
  return sequence % 2 == 1;
#endif
}

class ExecutorSpin final {
public:
  explicit ExecutorSpin(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {}

  ExecutorSpin(const ExecutorSpin &) = delete;
  ExecutorSpin & operator=(const ExecutorSpin &) = delete;

  ~ExecutorSpin()
  {
    try {
      stop();
    } catch (...) {
    }
  }

  void stop()
  {
    if (stopped_) {
      return;
    }
    stopped_ = true;
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::Executor & executor_;
  std::thread thread_;
  bool stopped_{false};
};

class CommonServer final : public rclcpp::Node {
public:
  CommonServer(const std::string & name, const std::string & service_name)
  : Node(name, node_options())
  {
    service_ = create_service<Service>(
      service_name,
      [this](
        const std::shared_ptr<Service::Request> request,
        std::shared_ptr<Service::Response> response) {
#ifdef RCLCPPYY_BENCHMARK_TRIGGER
        (void)request;
        constexpr bool value = true;
        response->success = true;
        response->message = "triggered";
#else
        const bool value = request->data;
        response->success = request->data;
        response->message = request->data ? "enabled" : "disabled";
#endif
        total_.fetch_add(1, std::memory_order_relaxed);
        true_total_.fetch_add(value ? 1 : 0, std::memory_order_relaxed);
        if (armed_.load(std::memory_order_acquire)) {
          measured_.fetch_add(1, std::memory_order_relaxed);
          true_measured_.fetch_add(value ? 1 : 0, std::memory_order_relaxed);
          checksum_.fetch_add(response_code(value, *response), std::memory_order_relaxed);
        }
      },
      service_qos());
  }

  void arm() {armed_.store(true, std::memory_order_release);}
  std::uint64_t total() const {return total_.load(std::memory_order_acquire);}
  std::uint64_t measured() const {return measured_.load(std::memory_order_acquire);}
  std::uint64_t true_total() const {return true_total_.load(std::memory_order_acquire);}
  std::uint64_t true_measured() const {return true_measured_.load(std::memory_order_acquire);}
  std::uint64_t checksum() const {return checksum_.load(std::memory_order_acquire);}

private:
  rclcpp::Service<Service>::SharedPtr service_;
  std::atomic<bool> armed_{false};
  std::atomic<std::uint64_t> total_{0};
  std::atomic<std::uint64_t> measured_{0};
  std::atomic<std::uint64_t> true_total_{0};
  std::atomic<std::uint64_t> true_measured_{0};
  std::atomic<std::uint64_t> checksum_{0};
};

int run_server(int argc, char ** argv)
{
  if (argc != 8) {
    throw std::invalid_argument(
            "server usage: server SERVICE NODE WARMUP MESSAGES TOKEN INTERFACE");
  }
  const std::string service_name(argv[2]);
  const std::string node_name(argv[3]);
  const auto warmup = parse_uint64(argv[4], "warmup");
  const auto messages = parse_uint64(argv[5], "messages");
  const std::string token(argv[6]);
  if (std::string(argv[7]) != kServiceType) {
    throw std::invalid_argument("server interface differs from compiled service type");
  }
  auto server = std::make_shared<CommonServer>(node_name, service_name);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  ExecutorSpin spin(executor);
  std::cout << kPrefix
            << "{\"schema\":\"rclcppyy.service-client-server-event/v1\","
            << "\"event\":\"ready\",\"run_token\":" << quote(token) << ","
            << "\"pid\":" << getpid() << ",\"process_group_id\":" << getpgrp() << ","
            << "\"node_name\":" << quote(node_name) << ",\"loaded_rmw\":"
            << quote(loaded_rmw()) << ",\"execution_model\":" << quote(kServerModel) << ","
            << "\"service_name\":" << quote(service_name) << ","
            << "\"service_type\":" << quote(kServiceType) << "}" << std::endl;

  std::string command;
  if (!std::getline(std::cin, command) || command != "START") {
    throw std::runtime_error("server expected START control");
  }
  if (server->total() != warmup) {
    throw std::runtime_error("server warmup count is invalid");
  }
  server->arm();
  const auto rss_baseline = peak_rss_bytes();
  const auto cpu_start = process_cpu_ns();
  std::cout << kPrefix
            << "{\"schema\":\"rclcppyy.service-client-server-event/v1\","
            << "\"event\":\"armed\",\"run_token\":" << quote(token) << ","
            << "\"pid\":" << getpid() << ",\"process_group_id\":" << getpgrp() << ","
            << "\"warmup_requests\":" << warmup << ","
            << "\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\",\"service_type\":"
            << quote(kServiceType) << "}" << std::endl;
  if (!std::getline(std::cin, command) || command != "REPORT") {
    throw std::runtime_error("server expected REPORT control");
  }
  const auto cpu_stop = process_cpu_ns();
  const auto rss_final = peak_rss_bytes();
  spin.stop();
  executor.remove_node(server);
  const auto total = server->total();
  const auto measured = server->measured();
  const auto true_total = server->true_total();
  const auto true_measured = server->true_measured();
  const auto checksum = server->checksum();
  server.reset();
  rclcpp::shutdown();
  const bool correct = total == warmup + messages && measured == messages;
  std::cout << kPrefix
            << "{\"schema\":\"rclcppyy.service-client-server-event/v1\","
            << "\"event\":\"report\",\"run_token\":" << quote(token) << ","
            << "\"service_type\":" << quote(kServiceType) << ","
            << "\"warmup_requests\":" << warmup << ",\"total_requests\":" << total << ","
            << "\"measured_requests\":" << measured << ",\"true_total\":"
            << true_total << ",\"true_measured\":" << true_measured << ","
            << "\"response_checksum\":" << checksum << ",\"exceptions\":0,"
            << "\"pending_requests\":0,\"cpu_time_ns\":" << (cpu_stop - cpu_start) << ","
            << "\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\",\"rss_guard\":"
            << rss_guard_json(rss_baseline, rss_final) << ",\"correct\":"
            << (correct ? "true" : "false") << ",\"teardown_clean\":true}" << std::endl;
  return correct ? 0 : 2;
}

std::shared_ptr<Service::Response> call_once(
  const std::shared_ptr<rclcpp::Node> & node,
  const rclcpp::Client<Service>::SharedPtr & client,
  bool value)
{
  auto request = std::make_shared<Service::Request>();
#ifndef RCLCPPYY_BENCHMARK_TRIGGER
  request->data = value;
#endif
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(15)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("client timed out waiting for a SetBool response");
  }
  auto response = future.get();
  validate_response(value, *response);
  return response;
}

void verify_server_graph(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & server_name,
  const std::string & service_name)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (std::chrono::steady_clock::now() < deadline) {
    if (node->count_services(service_name) == 1) {
      try {
        const auto services = node->get_service_names_and_types_by_node(server_name, "/");
        const auto found = services.find(service_name);
        if (found != services.end() && found->second.size() == 1 &&
          found->second[0] == kServiceType)
        {
          return;
        }
      } catch (const std::runtime_error &) {
      }
    } else if (node->count_services(service_name) > 1) {
      throw std::runtime_error("service graph contains more than one benchmark server");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("client timed out verifying the exact service graph");
}

int run_client(int argc, char ** argv)
{
  if (argc != 9) {
    throw std::invalid_argument(
            "client usage: client SERVICE CLIENT_NODE SERVER_NODE WARMUP MESSAGES TOKEN INTERFACE");
  }
  const std::string service_name(argv[2]);
  const std::string client_name(argv[3]);
  const std::string server_name(argv[4]);
  const auto warmup = parse_uint64(argv[5], "warmup");
  const auto messages = parse_uint64(argv[6], "messages");
  const std::string token(argv[7]);
  if (std::string(argv[8]) != kServiceType) {
    throw std::invalid_argument("client interface differs from compiled service type");
  }
  auto node = std::make_shared<rclcpp::Node>(client_name, node_options());
  auto client = node->create_client<Service>(service_name, service_qos());
  verify_server_graph(node, server_name, service_name);
  for (std::uint64_t sequence = 1; sequence <= warmup; ++sequence) {
    call_once(node, client, sequence_value(sequence));
  }
  std::cout << kPrefix
            << "{\"schema\":\"rclcppyy.service-client-client-event/v1\","
            << "\"event\":\"warmed\",\"variant\":\"aot-staged\","
            << "\"run_token\":" << quote(token) << ",\"pid\":" << getpid() << ","
            << "\"process_group_id\":" << getpgrp() << ",\"node_name\":"
            << quote(client_name) << ",\"loaded_rmw\":" << quote(loaded_rmw()) << ","
            << "\"execution_model\":\"conventional-release-aot-client\","
            << "\"client_authority\":\"cpp\","
            << "\"cache\":{\"state\":\"prebuilt\",\"kind\":\"aot-binary\"},"
            << "\"entity_type\":\"rclcpp::Client<" << kCppServiceType << ">\","
            << "\"warmup_requests\":" << warmup << ",\"topology_verified\":true,"
            << "\"endpoint_count\":1,\"server_node\":" << quote(server_name) << ","
            << "\"service_name\":" << quote(service_name) << ","
            << "\"service_type\":" << quote(kServiceType) << ",\"qos_verified\":true}" << std::endl;

  std::string command;
  if (!std::getline(std::cin, command) || command != "START") {
    throw std::runtime_error("client expected START control");
  }
  std::vector<std::uint64_t> latencies;
  latencies.reserve(messages);
  std::uint64_t checksum = 0;
  std::uint64_t true_requests = 0;
  const auto rss_baseline = peak_rss_bytes();
  const auto cpu_start = process_cpu_ns();
  const auto wall_start = std::chrono::steady_clock::now();
  for (std::uint64_t offset = 1; offset <= messages; ++offset) {
    const auto sequence = warmup + offset;
    const bool value = sequence_value(sequence);
    const auto started = std::chrono::steady_clock::now();
    const auto response = call_once(node, client, value);
    const auto stopped = std::chrono::steady_clock::now();
    checksum += response_code(value, *response);
    true_requests += value ? 1 : 0;
    latencies.push_back(static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(stopped - started).count()));
  }
  const auto wall_stop = std::chrono::steady_clock::now();
  const auto cpu_stop = process_cpu_ns();
  const auto rss_final = peak_rss_bytes();
  const auto elapsed = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(wall_stop - wall_start).count());
  std::cout << kPrefix
            << "{\"schema\":\"rclcppyy.service-client-client-event/v1\","
            << "\"event\":\"measured\",\"variant\":\"aot-staged\","
            << "\"run_token\":" << quote(token) << ",\"pid\":" << getpid() << ","
            << "\"service_type\":" << quote(kServiceType) << ","
            << "\"process_group_id\":" << getpgrp() << ",\"messages\":" << messages << ","
            << "\"total_requests\":" << (warmup + messages) << ","
            << "\"true_measured\":" << true_requests << ",\"response_checksum\":"
            << checksum << ",\"python_orchestration_requests_measured\":0,"
            << "\"python_request_crossings_measured\":0,"
            << "\"python_response_crossings_measured\":0,"
            << "\"python_message_conversions_measured\":0,"
            << "\"exceptions\":0,\"pending_requests\":0,\"elapsed_ns\":" << elapsed << ","
            << "\"cpu_time_ns\":" << (cpu_stop - cpu_start) << ","
            << "\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\",\"rss_guard\":"
            << rss_guard_json(rss_baseline, rss_final) << ",\"latency_ns\":[";
  for (std::size_t index = 0; index < latencies.size(); ++index) {
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << latencies[index];
  }
  std::cout << "]}" << std::endl;

  if (!std::getline(std::cin, command) || command != "TEARDOWN") {
    throw std::runtime_error("client expected TEARDOWN control");
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (node->count_services(service_name) != 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (node->count_services(service_name) != 0 || client->service_is_ready()) {
    throw std::runtime_error("server endpoint did not disappear before client teardown");
  }
  client.reset();
  node.reset();
  rclcpp::shutdown();
  std::cout << kPrefix
            << "{\"schema\":\"rclcppyy.service-client-client-event/v1\","
            << "\"event\":\"teardown\",\"variant\":\"aot-staged\","
            << "\"run_token\":" << quote(token) << ",\"pid\":" << getpid() << ","
            << "\"process_group_id\":" << getpgrp() << ","
            << "\"service_type\":" << quote(kServiceType) << ","
            << "\"endpoint_disappeared\":true,\"teardown_clean\":true}" << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: service_client_aot server|client ...\n";
    return 2;
  }
  try {
    rclcpp::init(0, nullptr);
    const std::string mode(argv[1]);
    if (mode == "server") {
      return run_server(argc, argv);
    }
    if (mode == "client") {
      return run_client(argc, argv);
    }
    throw std::invalid_argument("mode must be server or client");
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "service client AOT error: " << error.what() << '\n';
    return 2;
  }
}
