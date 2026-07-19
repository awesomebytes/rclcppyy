#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcppyy_test_interfaces/action/accumulate.hpp"
#include "rclcppyy_test_interfaces/msg/stamped_value.hpp"
#include "rclcppyy_test_interfaces/srv/transform_value.hpp"

class InteropPeer : public rclcpp::Node
{
public:
  using StampedValue = rclcppyy_test_interfaces::msg::StampedValue;
  using TransformValue = rclcppyy_test_interfaces::srv::TransformValue;
  using Accumulate = rclcppyy_test_interfaces::action::Accumulate;
  using GoalHandleAccumulate = rclcpp_action::ServerGoalHandle<Accumulate>;

  explicit InteropPeer(const std::string & prefix)
  : Node("rclcppyy_aot_interop_peer"),
    feedback_topic_(prefix + "/accumulate/_action/feedback")
  {
    reply_publisher_ = create_publisher<StampedValue>(prefix + "/cpp_to_python", 10);
    request_subscription_ = create_subscription<StampedValue>(
      prefix + "/python_to_cpp", 10,
      [this](const StampedValue::SharedPtr request) {
        StampedValue reply = *request;
        reply.sequence += 100;
        reply.label = "aot:" + request->label;
        for (auto & sample : reply.samples) {
          sample *= 2;
        }
        reply_publisher_->publish(reply);
        std::cout << "AOT_MESSAGE_ROUNDTRIP_OK" << std::endl;
      });

    transform_service_ = create_service<TransformValue>(
      prefix + "/transform",
      [](const TransformValue::Request::SharedPtr request,
      TransformValue::Response::SharedPtr response) {
        response->output = request->input;
        response->output.sequence += 1;
        response->output.label = "aot-service:" + request->input.label;
        for (auto & sample : response->output.samples) {
          sample *= request->scale;
        }
        std::cout << "AOT_SERVICE_ROUNDTRIP_OK" << std::endl;
      });

    action_server_ = rclcpp_action::create_server<Accumulate>(
      this,
      prefix + "/accumulate",
      std::bind(
        &InteropPeer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&InteropPeer::handle_cancel, this, std::placeholders::_1),
      std::bind(&InteropPeer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Accumulate::Goal> goal)
  {
    if (goal->target < 1 || goal->target > 100) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAccumulate>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAccumulate> goal_handle)
  {
    std::thread(
      [this, goal_handle]() {
        const auto discovery_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (count_subscribers(feedback_topic_) == 0 &&
        std::chrono::steady_clock::now() < discovery_deadline)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (count_subscribers(feedback_topic_) == 0) {
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Accumulate::Feedback>();
        int64_t total = 0;
        for (int32_t current = 1; current <= goal->target; ++current) {
          if (goal_handle->is_canceling()) {
            auto canceled = std::make_shared<Accumulate::Result>();
            canceled->total = total;
            goal_handle->canceled(canceled);
            return;
          }
          total += current;
          feedback->current = current;
          feedback->partial_total = total;
          goal_handle->publish_feedback(feedback);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto result = std::make_shared<Accumulate::Result>();
        result->total = total;
        result->summary.sequence = goal->target;
        result->summary.label = "aot-action";
        result->summary.samples = {goal->target, static_cast<int32_t>(total)};
        goal_handle->succeed(result);
        std::cout << "AOT_ACTION_ROUNDTRIP_OK" << std::endl;
      }).detach();
  }

  rclcpp::Publisher<StampedValue>::SharedPtr reply_publisher_;
  rclcpp::Subscription<StampedValue>::SharedPtr request_subscription_;
  rclcpp::Service<TransformValue>::SharedPtr transform_service_;
  std::string feedback_topic_;
  rclcpp_action::Server<Accumulate>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: interop_peer /absolute/prefix" << std::endl;
    return 2;
  }
  rclcpp::init(argc, argv);
  auto peer = std::make_shared<InteropPeer>(argv[1]);
  std::cout << "AOT_PEER_READY" << std::endl;
  rclcpp::spin(peer);
  rclcpp::shutdown();
  std::cout << "AOT_PEER_STOPPED" << std::endl;
  return 0;
}
