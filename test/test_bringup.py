#!/usr/bin/env python3

import unittest
import cppyy

from rclcppyy.bringup_rclcpp import bringup_rclcpp


class TestBringup(unittest.TestCase):

    def test_bringup_rclcpp(self):
        # Should not raise any exceptions
        bringup_rclcpp()
        
        # After bringing up rclcpp, we should be able to access rclcpp classes
        # through cppyy's global namespace
        node_attributes = dir(cppyy.gbl.rclcpp.Node)
        
        # The Node class should have some basic expected attributes
        self.assertIn('get_name', node_attributes)
        self.assertIn('get_namespace', node_attributes)
        

if __name__ == '__main__':
    unittest.main()
