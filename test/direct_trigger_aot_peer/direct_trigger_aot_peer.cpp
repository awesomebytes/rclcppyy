#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{

using namespace std::chrono_literals;
using Service = std_srvs::srv::Trigger;

int run_client(const std::string & service_name)
{
  auto node = std::make_shared<rclcpp::Node>("direct_trigger_aot_client");
  auto client = node->create_client<Service>(service_name);
  if (!client->wait_for_service(10s)) {
    std::cerr << "AOT Trigger client service timeout\n";
    return 10;
  }
  auto future = client->async_send_request(std::make_shared<Service::Request>());
  if (rclcpp::spin_until_future_complete(node, future, 10s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cerr << "AOT Trigger client response timeout\n";
    return 11;
  }
  const auto response = future.get();
  if (!response->success || response->message != "direct-trigger-aot-response") {
    std::cerr << "AOT Trigger client unexpected response: " << response->message << '\n';
    return 12;
  }
  std::cout << "AOT_TRIGGER_CLIENT_OK" << std::endl;
  return 0;
}

int run_server(const std::string & service_name)
{
  auto node = std::make_shared<rclcpp::Node>("direct_trigger_aot_server");
  std::atomic<bool> handled{false};
  auto service = node->create_service<Service>(
    service_name,
    [&handled](
      const std::shared_ptr<Service::Request>,
      std::shared_ptr<Service::Response> response) {
      response->success = true;
      response->message = "aot-trigger-response";
      handled.store(true, std::memory_order_release);
    });
  (void)service;
  std::cout << "AOT_TRIGGER_SERVER_READY" << std::endl;
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (
    rclcpp::ok() && !handled.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(1ms);
  }
  if (!handled.load(std::memory_order_acquire)) {
    std::cerr << "AOT Trigger server request timeout\n";
    return 20;
  }
  std::cout << "AOT_TRIGGER_SERVER_OK" << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: direct_trigger_aot_peer client|server SERVICE_NAME\n";
    return 2;
  }
  rclcpp::init(0, nullptr);
  int result = 0;
  try {
    const std::string mode = argv[1];
    if (mode == "client") {
      result = run_client(argv[2]);
    } else if (mode == "server") {
      result = run_server(argv[2]);
    } else {
      std::cerr << "unknown mode: " << mode << '\n';
      result = 3;
    }
  } catch (const std::exception & exception) {
    std::cerr << "AOT Trigger peer exception: " << exception.what() << '\n';
    result = 4;
  }
  rclcpp::shutdown();
  std::cout << "AOT_TRIGGER_TEARDOWN_OK" << std::endl;
  return result;
}
