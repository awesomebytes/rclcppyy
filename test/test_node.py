#!/usr/bin/env python3

import unittest

import rclpy
from rclcppyy import Node


class TestNode(unittest.TestCase):
    
    def setUp(self):
        rclpy.init()
    
    def tearDown(self):
        rclpy.shutdown()
        
    def test_node_creation(self):
        node = Node('test_node')
        self.assertEqual(node.get_name(), 'test_node')
        node.destroy_node()


if __name__ == '__main__':
    unittest.main() 