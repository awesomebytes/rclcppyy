#!/usr/bin/env python3
"""Phase 0 research probe for a same-handle rclcpp publisher.

This is deliberately not a production API.  It tests one architectural fact:
an ``rclcpp::Publisher<T>`` can be constructed against the ``rcl_node_t`` owned
by a stock ``rclpy.Node`` without creating an ``rclcpp::Node`` sidecar.

The bridge below relies on private/ABI-sensitive details:

* ``node.handle.pointer`` exposes a native address, not a stable public adoption API.
* The adapter returns non-owning ``shared_ptr<rcl_node_t>`` instances.  The Python
  node must outlive every C++ entity, and the C++ entities must be released first.
* The adapter has no compatible ``rclcpp::Context``.  Intra-process transport,
  executors, callback groups, events, and topic statistics are intentionally absent.
* Only typed publishing is exercised.  This says nothing about safely adopting
  subscriptions, timers, services, clients, or their executor integration.

Run this only as an isolated process.  The pytest wrapper asserts the emitted
markers and the process's normal, fault-free exit.
"""
import gc
import os
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rclcppyy.bringup_rclcpp import bringup_rclcpp, shutdown_rclcpp


SPIN_DEADLINE_S = 15.0
NAMESPACE = "/rclcppyy_phase0"
TOPIC = "/rclcppyy_phase0/same_handle"
PAYLOAD = "same-handle-publisher-ok"


_BORROWED_NODE_BASE = r"""
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace rclcppyy_phase0_probe
{

// Minimal adapter used only by this probe.  It never owns or finalizes node_.
class BorrowedNodeBase final : public rclcpp::node_interfaces::NodeBaseInterface
{
public:
  explicit BorrowedNodeBase(rcl_node_t * node) : node_(node) {}

  const char * get_name() const override {return rcl_node_get_name(node_);}
  const char * get_namespace() const override {return rcl_node_get_namespace(node_);}
  const char * get_fully_qualified_name() const override
  {
    return rcl_node_get_fully_qualified_name(node_);
  }

  rclcpp::Context::SharedPtr get_context() override {return nullptr;}
  rcl_node_t * get_rcl_node_handle() override {return node_;}
  const rcl_node_t * get_rcl_node_handle() const override {return node_;}

  std::shared_ptr<rcl_node_t> get_shared_rcl_node_handle() override
  {
    return std::shared_ptr<rcl_node_t>(node_, [](rcl_node_t *) {});
  }

  std::shared_ptr<const rcl_node_t> get_shared_rcl_node_handle() const override
  {
    return std::shared_ptr<const rcl_node_t>(node_, [](const rcl_node_t *) {});
  }

  rclcpp::CallbackGroup::SharedPtr create_callback_group(
    rclcpp::CallbackGroupType, bool) override
  {
    return nullptr;
  }

  rclcpp::CallbackGroup::SharedPtr get_default_callback_group() override {return nullptr;}
  bool callback_group_in_node(rclcpp::CallbackGroup::SharedPtr) override {return false;}
  void for_each_callback_group(const CallbackGroupFunction &) override {}
  std::atomic_bool & get_associated_with_executor_atomic() override {return associated_;}

  rclcpp::GuardCondition & get_notify_guard_condition() override
  {
    throw std::runtime_error("guard conditions are outside this Phase 0 probe");
  }

  rclcpp::GuardCondition::SharedPtr get_shared_notify_guard_condition() override
  {
    return nullptr;
  }

  void trigger_notify_guard_condition() override {}
  bool get_use_intra_process_default() const override {return false;}
  bool get_enable_topic_statistics_default() const override {return false;}

  std::string resolve_topic_or_service_name(
    const std::string & name, bool, bool) const override
  {
    // Python resolves the absolute topic before calling this probe factory.
    return name;
  }

private:
  rcl_node_t * node_;
  std::atomic_bool associated_{false};
};

using StringPublisher = rclcpp::Publisher<std_msgs::msg::String>;

std::shared_ptr<StringPublisher> make_string_publisher(
  uintptr_t node_address, const std::string & topic)
{
  auto base = std::make_shared<BorrowedNodeBase>(
    reinterpret_cast<rcl_node_t *>(node_address));
  auto options = rclcpp::PublisherOptions();
  auto qos = rclcpp::QoS(10);
  auto publisher = std::make_shared<StringPublisher>(base.get(), topic, qos, options);
  publisher->post_init_setup(base.get(), topic, qos, options);
  return publisher;
}

}  // namespace rclcppyy_phase0_probe
"""


def _wait_for_graph_and_match(node, publisher, node_name, executor):
    deadline = time.monotonic() + SPIN_DEADLINE_S
    expected_identity = (node_name, NAMESPACE)

    while time.monotonic() < deadline:
        identities = node.get_node_names_and_namespaces()
        endpoint_info = node.get_publishers_info_by_topic(TOPIC)
        endpoint_identities = [(info.node_name, info.node_namespace) for info in endpoint_info]
        if (identities.count(expected_identity) == 1 and
                endpoint_identities == [expected_identity] and
                publisher.get_subscription_count() >= 1):
            return identities, endpoint_identities
        executor.spin_once(timeout_sec=0.05)

    raise AssertionError(
        "same-handle publisher did not converge: "
        f"nodes={identities!r}, endpoints={endpoint_identities!r}, "
        f"subscription_count={publisher.get_subscription_count()}")


def main():
    # Bring in rclcpp types, but do not initialize an rclcpp context or create an
    # rclcpp node.  The custom rclpy context below is the sole context authority.
    bringup_rclcpp()
    import cppyy
    cppyy.include("std_msgs/msg/string.hpp")
    cppyy.cppdef(_BORROWED_NODE_BASE)

    from std_msgs.msg import String

    context = Context()
    context.init(args=[])
    node = None
    executor = None
    subscription = None
    publisher = None

    try:
        node_name = f"same_handle_authority_{os.getpid()}"
        node = rclpy.create_node(node_name, namespace=NAMESPACE, context=context)

        assert type(node) is Node, f"expected an exact stock Node, got {type(node)!r}"
        assert node.context is context
        assert context.ok()
        assert not rclpy.ok(), "the default rclpy context must remain uninitialized"
        print("PHASE0_AUTHORITY_OK", flush=True)

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        received = []
        subscription = node.create_subscription(
            String, TOPIC, lambda message: received.append(message.data), 10)

        with node.handle:
            publisher = cppyy.gbl.rclcppyy_phase0_probe.make_string_publisher(
                node.handle.pointer, node.resolve_topic_name(TOPIC))

        identities, endpoint_identities = _wait_for_graph_and_match(
            node, publisher, node_name, executor)
        requested_identity = (node_name, NAMESPACE)
        assert identities.count(requested_identity) == 1
        assert not any(name == f"{node_name}_rclcpp" for name, _ in identities)
        assert endpoint_identities == [requested_identity]
        print("PHASE0_GRAPH_OK", flush=True)

        cpp_message = cppyy.gbl.std_msgs.msg.String()
        cpp_message.data = PAYLOAD
        publisher.publish(cpp_message)

        deadline = time.monotonic() + SPIN_DEADLINE_S
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert received == [PAYLOAD], f"payload mismatch: {received!r}"
        print("PHASE0_ROUNDTRIP_OK", flush=True)
    finally:
        # This order is the central lifetime invariant of the experiment.  The
        # rclcpp publisher stores a non-owning node handle, so finalize it while
        # the authoritative rclpy node and context are still valid.
        publisher = None
        gc.collect()
        if node is not None:
            if executor is not None:
                executor.remove_node(node)
            if subscription is not None:
                node.destroy_subscription(subscription)
            node.destroy_node()
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        context.try_shutdown()
        shutdown_rclcpp()

    print("PHASE0_TEARDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
