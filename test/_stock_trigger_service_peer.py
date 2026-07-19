#!/usr/bin/env python3
"""Stock-rclpy Trigger service and client peer for direct-C++ interop."""

import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


if len(sys.argv) != 3:
    raise SystemExit("usage: peer.py STOCK_SERVICE DIRECT_SERVICE")

stock_service_name, direct_service_name = sys.argv[1:]
rclpy.init(args=[])
node = Node("stock_trigger_service_peer")
server_handled = False


def serve(_request, response):
    global server_handled
    response.success = True
    response.message = "stock-rclpy-response"
    server_handled = True
    return response


service = node.create_service(Trigger, stock_service_name, serve)
client = node.create_client(Trigger, direct_service_name)
print("STOCK_TRIGGER_READY", flush=True)
assert client.wait_for_service(timeout_sec=10.0)
future = client.call_async(Trigger.Request())
deadline = time.monotonic() + 20.0
while (not server_handled or not future.done()) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.02)
assert server_handled
assert future.done() and future.exception() is None
assert future.result().success is True
assert future.result().message == "direct-cpp-response"
print("STOCK_TRIGGER_BIDIRECTIONAL_OK", flush=True)
node.destroy_client(client)
node.destroy_service(service)
node.destroy_node()
rclpy.shutdown()
print("STOCK_TRIGGER_TEARDOWN_OK", flush=True)
